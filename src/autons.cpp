#include "vex.h"

// ----------------------------------------------------------------------------
//             helper Functions
// ----------------------------------------------------------------------------



// ----------------------------------------------------------------------------
//             Auton Functions
// ----------------------------------------------------------------------------

void left_auton(){
  chassis.setHeading(0);
  // sequence of actions for left auton
}

void right_auton(){ 
  chassis.setHeading(-90);
  // sequence of actions for right auton
}

// The names of the autonomous routines to be displayed in the menu.
char const * autonMenuText[] = {
  "Left",
  "Right"
};


// Runs the selected autonomous routine.
void runAutonItem() {
  switch (currentAutonSelection) {
  case 0:
    left_auton();
    break;  
  case 1:
    right_auton();
    break;  
  }
}

int skillStep = 0;  // Current step in the skill routine. if skillStep > 0, the robot is in skill mode.
void run_skill_step() {
  switch (skillStep) {
    case 1:
      // Add actions for the first skill step here.
      break;
    case 2:
      // Add actions for the second skill step here.
      break;
  }
}

// ----------------------------------------------------------------------------
//                 Menu display and other system functions
// ----------------------------------------------------------------------------
int autonNum;                         // Total number of autons, automatically calculated based on the size of the autonMenuText array.
int currentAutonSelection = 0;        // Current auton selection.
bool autonTestMode = false;           // Indicates if in test mode.
bool exitAutonMenu = false;           // Flag to exit the autonomous menu.
bool endGameTimerEnabled = false;      // Flag to indicate if endgame timer is enabled.
const int END_GAME_SECONDS = 85;      // Endgame reminder starts at 85 seconds.

// This function is called when the autonomous period starts.
void autonomous(void) {
  // Exits the autonomous menu.
  exitAutonMenu = true;
  endGameTimerEnabled = true;

  additionalSetupAuton();

  if (skillStep > 0){ // in skill mode
    skill();
    return;
  }

  // Runs the selected autonomous routine.
  runAutonItem();
}

int teamColor = 0; // 0: off, 1: red, 2: blue
void changeTeamColor() {
//  if (!autonTestMode) return; //uncomment the line of code for the final code in case accidentally press button
  controller1.rumble("-");
  teamColor = (teamColor + 1) % 3;
  if (teamColor == 0) printControllerScreen("No color selected");
  else if (teamColor == 1) printControllerScreen("Team red");
  else if (teamColor == 2) printControllerScreen("Team blue");
  showMotorTemperature();
}

// Prints the selected autonomous routine to the Brain and Controller screens.
void printMenuItem() {
  // Clears the Brain screen.
  if (teamColor == 1) 
  {
    Brain.Screen.clearScreen(color::red);
  }
  else if (teamColor == 2) 
  {
    Brain.Screen.clearScreen(color::blue);
  }
  else
  {
    Brain.Screen.clearScreen(color::black);
  }
  // Sets the cursor to the second row, second column.
  Brain.Screen.setCursor(2, 2);
  currentAutonSelection = currentAutonSelection % autonNum;

  if (skillStep > 0)
  {
    Brain.Screen.print("skill");
    printControllerScreen("skill");
    return; // Skip printing the selected autonomous routine if in skill mode.
  }
  // Prints the selected autonomous routine name.
  Brain.Screen.print("%s", autonMenuText[currentAutonSelection]);
  printControllerScreen(autonMenuText[currentAutonSelection]);
}

// Displays the autonomous menu on the Brain screen.
void showAutonMenu() {
  autonNum = sizeof(autonMenuText) / sizeof(autonMenuText[0]);

  Brain.Screen.setFont(mono40);
  printMenuItem();

  // This loop runs until the autonomous menu is exited.
  while (!exitAutonMenu) {
    if (Brain.Screen.pressing()) {
      // Waits until the finger is lifted from the screen.
      while (Brain.Screen.pressing()) {
        wait(20, msec);
      }

      if (Brain.Screen.yPosition() < 120)
      {
        // Cycles through the autonomous routines.
        currentAutonSelection = (currentAutonSelection + 1) % autonNum;
        controller1.rumble(".");
      }
      else 
      {
        changeTeamColor();
      }
      printMenuItem();
    }

    Brain.Screen.setCursor(5, 1);
    Brain.Screen.print("Heading : %.1f", chassis.getHeading());

    // This wait prevents the loop from using too much CPU time.
    wait(50, msec);
  }
  Brain.Screen.setFont(mono20);
}

// This function is a thread that runs in the background to remind the driver of the end game.
void endgameTimer() {
  // Waits until the end game starts.
  while (Brain.Timer.time(sec) < END_GAME_SECONDS) {
    wait(500, msec);
  }
  if (endGameTimerEnabled)
  {
    printControllerScreen("end game");
    controller1.rumble("-");
  }

  // Checks motor health every 60 seconds during drive practice.
  while(true)
  {
    wait(60, seconds);
   // if (!autonTestMode) checkMotors(NUMBER_OF_MOTORS);
  }
}

void exitAuton()
{
  exitAutonMenu = true;
    // Clears the Brain timer.
  Brain.Timer.clear();
    // Starts the end game timer thread.
  thread endgameTimer_thread = thread(endgameTimer);
}

bool setupinertialSensor() {
  if (INERTIAL_PORT < 0) return true;
  wait(100, msec);
  if (!chassis.inertialSensor.installed()) {
    printControllerScreen("inertial failure", true);
    return false;  
  }

  chassis.inertialSensor.calibrate(3);
  // Waits until the inertial sensor is calibrated.
  while (chassis.inertialSensor.isCalibrating()) {
    wait(100, msec);
  }
  // Rumbles the controller to indicate that the inertial sensor is calibrated.
  printControllerScreen("gyro calibrated");
  controller1.rumble(".");

  chassis.setGyroScale(INERTIAL_SCALE);
  return true;
}

void loadConfigParameters()
{
  // Load parameters from the SD card.
  if (Brain.SDcard.isInserted()) {
    if (Brain.SDcard.exists("parameters.txt")) {  
      // Open the file for reading.
      uint8_t  myReadBuffer[1000];  
      Brain.SDcard.loadfile("parameters.txt", myReadBuffer, sizeof(myReadBuffer));
      wait(0.5, seconds);

      char line_buffer[256];
      char* buffer_ptr = (char*)myReadBuffer;
      char* line_end;

      // Process the buffer line by line.
      while ((line_end = strchr(buffer_ptr, '\n')) != NULL) {
          // Copy the current line into the line_buffer.
          int line_len = line_end - buffer_ptr;
          strncpy(line_buffer, buffer_ptr, line_len);
          line_buffer[line_len] = '\0';

          // Find the position of the '=' character.
          char* equals_sign = strchr(line_buffer, '=');
          if (equals_sign != NULL) {
              // Null-terminate the key part of the string.
              *equals_sign = '\0';
              
              // Extract the key and value strings.
              char* key = trim_whitespace(line_buffer);
              char* value_str = trim_whitespace(equals_sign + 1);

              // Check if the line is for 'auton' or 'drive_mode'.
              if (strcmp(key, "auton") == 0) {
                  currentAutonSelection = atoi(value_str);
              } else if (strcmp(key, "drive_mode") == 0) {
                  drive_mode = atoi(value_str);
              }
              else if (strcmp(key, "color_sort") == 0) {
                  teamColor = atoi(value_str);
                  if(teamColor!=0) printControllerScreen(teamColor==1?"team red":"team blue");
                  wait(0.5, seconds);
              }
          }
          
          // Move the pointer to the start of the next line.
          buffer_ptr = line_end + 1;
      }
    }
  }
}

void saveConfigParameters()
{
  // Save parameters to the SD card.
  if (Brain.SDcard.isInserted()) {    
    // Create the parameter string with current values.
    char parameter_buffer[256];
    sprintf(parameter_buffer, "auton = %d\ndrive_mode = %d\ncolor_sort = %d\n", currentAutonSelection, drive_mode, teamColor);
    
    // Save the parameters to the SD card.
    int32_t result = Brain.SDcard.savefile("parameters.txt", (uint8_t*)parameter_buffer, strlen(parameter_buffer));
    
    if (result < 0) {
      printControllerScreen("save failed");
    }
    wait(0.5, seconds);
  } 
} 

// This function is called before the autonomous period starts.
void pre_auton() {
  // Sets up the inertial sensor.
  bool inertialSensorSetupSuccess = setupinertialSensor();

  bool motorsSetupSuccess = true;
  // Checks the motors.
  motorsSetupSuccess = checkMotors(NUMBER_OF_MOTORS);
  // Set the parameters for the chassis.
  setChassisDefaults();
  // Load parameters from the SD card.
  if (drive_mode >= 0) loadConfigParameters();
  // Shows the autonomous menu.
  if(inertialSensorSetupSuccess && motorsSetupSuccess) 
  {
    showAutonMenu();
    registerAutonTestButtons();
  }
}

// ----------------------------------------------------------------------------
//                 For Testing Autonomous and Configuration
// ----------------------------------------------------------------------------

void autonTestButtonCheck()
{
    // Enter auton test mode if Button A is held down within 5 seconds of driver control.
  while (Brain.Timer.time(sec) < 5 && !autonTestMode) {
    if (controller1.ButtonA.pressing())
    {
      wait(500, msec);
      if (controller1.ButtonA.pressing())
      {
        controller1.rumble("-");
        printControllerScreen("Test mode: ON");
        waitUntil(!controller1.ButtonA.pressing());
        showAutonMenu();
        autonTestMode = true;
        wait(1, seconds);
        break;
      }
    }
    wait(50, msec);
  }
  while(true)
  {
    if (autonTestMode) 
    {
      if(controller1.ButtonX.pressing())
      {
        waitUntil(!controller1.ButtonX.pressing());
        controller1.rumble(".");
        changeDriveMode();
      }
      if(controller1.ButtonRight.pressing())
      {
        waitUntil(!controller1.ButtonRight.pressing());
        controller1.rumble(".");
        // Scroll through the auton menu.
        currentAutonSelection = (currentAutonSelection + 1) % autonNum;
        saveConfigParameters();
        showAutonMenu();
      }
      if(controller1.ButtonLeft.pressing())
      {
        waitUntil(!controller1.ButtonLeft.pressing());
        controller1.rumble(".");
        // Scroll through the auton menu.
        changeTeamColor();
        saveConfigParameters();
        showAutonMenu();
      }
      if(controller1.ButtonA.pressing())
      {
        waitUntil(!controller1.ButtonA.pressing());
        // Run the selected autonomous routine for testing and display the run time.
        controller(primary).rumble(".");
        double t1 = Brain.Timer.time(sec);
        runAutonItem(); 
        double t2 = Brain.Timer.time(sec);
        char timeMsg[30];
        sprintf(timeMsg, "run time: %.1f", t2-t1);
        printControllerScreen(timeMsg);
        chassis.stop(coast);
      }
    } 
    wait(100, msec);
  }
}

void skill()
{
  // sequence of actions for skill mode
  while(controller1.ButtonX.pressing())  // if button X is held down, run the next skill step
  {
    run_skill_step();
    skillStep++;
  }
}


// Check the controller button press for autonomous testing.
void registerAutonTestButtons()
{
  if (!endGameTimerEnabled)
  {
    thread autonTestButtonThread = thread(autonTestButtonCheck);
  }
  if (skillStep > 0)
  {
    controller1.ButtonX.pressed(skill);
  }
}


