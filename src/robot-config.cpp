#include "vex.h"
using namespace vex;

// ------------------------------------------------------------------------
//               Drivetrain configuration
// ------------------------------------------------------------------------
brain Brain;
controller controller1 = controller(primary);

motor leftMotor1 = motor(numberToPort(LEFT_PORT1), ratio18_1, port_reverse(LEFT_PORT1));
motor leftMotor2 = motor(numberToPort(LEFT_PORT2), ratio6_1, port_reverse(LEFT_PORT2));
motor leftMotor3 = motor(numberToPort(LEFT_PORT3), ratio6_1, port_reverse(LEFT_PORT3));

motor rightMotor1 = motor(numberToPort(RIGHT_PORT1), ratio18_1, port_reverse(RIGHT_PORT1));
motor rightMotor2 = motor(numberToPort(RIGHT_PORT2), ratio6_1, port_reverse(RIGHT_PORT2));
motor rightMotor3 = motor(numberToPort(RIGHT_PORT3), ratio6_1, port_reverse(RIGHT_PORT3));

//uses middle motor encoder for position bc first one maybe 5.5 watt
Drivetrain chassis(
  //Left Motors:
  motor_group(leftMotor2, leftMotor1, leftMotor3),
  //Right Motors:
  motor_group(rightMotor2, rightMotor1, rightMotor3),
  //Inertial Sensor:
  inertial(numberToPort(INERTIAL_PORT))
);


// 0: single arcade drive, 1: double arcade, 2: tank drive
int drive_mode = DRIVE_MODE;
void changeDriveMode(){
  controller1.rumble("-");
  drive_mode = (drive_mode +1)%3;
    switch (drive_mode) {
    case 0:
      printControllerScreen("Single Arcade");
      break;
    case 1:
      printControllerScreen("Double Arcade");
      break;
    case 2:
      printControllerScreen("Tank Drive");
      break;
    }
  saveConfigParameters();
  showMotorTemperature();
}

// This is the user control function.
// It is called when the driver control period starts.
void usercontrol(void) {
  // Exits the autonomous menu.
  exitAuton();
  additionalSetupDriver();

  double throttle;
  double turn;
  double leftSpeed, rightSpeed;

  // This loop runs forever, controlling the robot during the driver control period.
  while (1) {
    float throttleLimit = speedLimitWithHeight();
    float turnLimit = sqrt(throttleLimit)*10;
    switch (drive_mode) {  
    case 1: // double arcade
      throttle = threshold(controller1.Axis2.position(), -throttleLimit, throttleLimit);
      turn = threshold(controller1.Axis4.position(), -turnLimit, turnLimit);
      if (LEFT_RIGHT_STICK_SWAP) {
        throttle = threshold(controller1.Axis3.position(), -throttleLimit, throttleLimit);
        turn = threshold(controller1.Axis1.position(), -turnLimit, turnLimit);
      }
      chassis.controlArcade(throttle, turn);
      break;
    case 0: // single arcade
      throttle = threshold(controller1.Axis2.position(), -throttleLimit, throttleLimit);
      turn = threshold(controller1.Axis1.position(), -turnLimit, turnLimit);
      if (LEFT_RIGHT_STICK_SWAP) {
        throttle = threshold(controller1.Axis3.position(), -throttleLimit, throttleLimit);
        turn = threshold(controller1.Axis4.position(), -turnLimit, turnLimit);
      }
      chassis.controlArcade(throttle, turn);
      break;
    case 2: // tank drive
      leftSpeed = threshold(controller1.Axis3.position(), -throttleLimit, throttleLimit);
      rightSpeed = threshold(controller1.Axis2.position(), -throttleLimit, throttleLimit);
      chassis.controlTank(leftSpeed, rightSpeed);    
      break;
    }
    
    if(!endGameTimerEnabled){
      Brain.Screen.setCursor(10, 1);
      Brain.Screen.print("Heading : %.1f    ", chassis.getHeading());
    }
    
    // This wait prevents the loop from using too much CPU time.
    wait(20, msec);
  } 
}


// ------------------------------------------------------------------------
//        Other subsystems: motors, & sensors
// ------------------------------------------------------------------------
motor grabberMotor = motor(numberToPort(GRABBER_PORT), ratio18_1, port_reverse(GRABBER_PORT));
motor liftMotor1 = motor(numberToPort(LIFT_PORT1), ratio18_1, port_reverse(LIFT_PORT1));
motor liftMotor2 = motor(numberToPort(LIFT_PORT2), ratio18_1, port_reverse(LIFT_PORT2));
motor_group liftMotor = motor_group(liftMotor1, liftMotor2);
rotation liftSensor = rotation(numberToPort(LIFT_SENSOR_PORT), port_reverse(LIFT_SENSOR_PORT));

motor rollerMotor = motor(numberToPort(ROLLER_PORT), ratio18_1, port_reverse(ROLLER_PORT));
optical rollerOptical = optical(numberToPort(OPTICAL_PORT));

// ------------------------------------------------------------------------
//        initialization functions
// ------------------------------------------------------------------------
void additionalSetupAuton() {// does action right when auton starts for all autons
 // raiseLiftAboveGround();
  grabberMotor.stop(hold);
  liftMotor.setPosition(0, degrees);
}

void additionalSetupDriver() {// does action right when driver control period starts
  grabberMotor.stop(coast);
  liftMotor.setStopping(coast);
  liftMotor.stop(coast);
}

void setChassisDefaults()
{
  // Set drivetrain parameters if needed: state of drivetrain, defult wheel parameter, pid parameters, etc.
  chassis.setWheelSettings(WHEEL_DIAMETER, GEAR_RATIO);
  chassis.setArcadeConstants(0.5, 0.5, 0.95);
 // chassis.setDistanceSensors(1, distance(numberToPort(FRONT_DISTANCE_PORT)));
 // chassis.setDistanceSensors(3, distance(numberToPort(BACK_DISTANCE_PORT)));
  liftMotor.setPosition(0, degrees);
  if (liftSensor.installed()) {
    liftSensor.setPosition(0, degrees);
  }
}

// ------------------------------------------------------------------------
//        Helper functions definition
// ------------------------------------------------------------------------

float getLiftPosition(bool printPosition) {
  float p = liftMotor.position(degrees); //If no rotation sensor use motor encoder
  if ( liftSensor.installed()){
    p = liftSensor.position(degrees);
  }
  if (printPosition) {
    char msg[30];
    sprintf(msg, "lift position: %.1f", p);
    printControllerScreen(msg);
  }
  return p;
}

float speedLimitWithHeight()
{
  if (LIFT_MAX_HEIGHT <= 0) return 100.0; // if no speed limit based on height

  int height = getLiftPosition();
  if (height <= LIFT_MAX_HEIGHT) {
    return 100.0 - (height * (100-SPEED_LIMIT)/ LIFT_MAX_HEIGHT);
  } else {
    return SPEED_LIMIT;
  } 
}

bool liftAtMax()
{
  if (LIFT_MAX_HEIGHT <= 0) return false; // if no max lift position 
  float height = getLiftPosition();

  if (height > LIFT_MAX_HEIGHT) {
    liftMotor.stop(hold);
    return true;
  }
  return false;
}

bool liftAtMin()
{
  if (LIFT_MAX_HEIGHT <= 0) return false; // no ground position 
  float height = getLiftPosition();
  if (height > 0) {
    return false;
  }
  liftMotor.stop(coast);
  return true;
}

void raiseLiftAboveGround()
{
    while(getLiftPosition() < 100){
      liftMotor.spin(forward, 12, volt);
      wait(30, msec);
    }
    liftMotor.stop(hold);
}

void dropLiftToGround()
{
  while(!liftAtMin()){
    liftMotor.spin(forward, -8, volt); // let the weight pull the lift down
    wait(30, msec);
  } 
}

// ------------------------------------------------------------------------
//              Button controls
// ------------------------------------------------------------------------

// todo: lift control
void buttonUpFunction() {
}

void buttonDownFunction() {
}


//Kevin's function for grabber motor 10/15/26
void buttonL1Function() {
  grabberMotor.spin(forward, 8, volt);
  wait(500, msec);
  grabberMotor.stop(coast);
  while(controller1.ButtonL1.pressing()) {
     wait(20, msec);
  }
  grabberMotor.spin(reverse, 8, volt);
  wait(500, msec);
  grabberMotor.stop(coast);
}

//Kevin's function for roller motor 10/15/26
void buttonR1Function() {
 while (controller1.ButtonR1.pressing()) {
   rollerMotor.spin(forward, 12, volt);
 }
 rollerMotor.stop(coast);
}


// toggle drivetrain brake, e.g. when protecting the roller
bool drivetrainBrakeOn = false;
void buttonBFunction (){
  controller1.rumble("-");
  printControllerScreen("Toggle brake");
  drivetrainBrakeOn = !drivetrainBrakeOn;
  if (drivetrainBrakeOn == true){
    chassis.stop(hold);
  }
  else {
    chassis.stop(coast);
  }
}

void setupButtonMapping() {
  controller1.ButtonUp.pressed(buttonUpFunction);
  controller1.ButtonDown.pressed(buttonDownFunction);

  controller1.ButtonL1.pressed(buttonL1Function);
  controller1.ButtonR1.pressed(buttonR1Function);

  controller1.ButtonA.pressed(quickTestCode);
  controller1.ButtonB.pressed(buttonBFunction);

}


// ------------------------------------------------------------------------
//              Quick Test Code
// ------------------------------------------------------------------------

// Add any code here for quick testing during driver control.
void quickTestCode()
{
   // Disable button if in auton test mode or in competition mode.
  if (autonTestMode  || endGameTimerEnabled) return; 
  wait(300, msec);
  if (controller1.ButtonA.pressing()) return; // Only trigger single press.
  additionalSetupAuton();
  controller1.rumble("-");
  chassis.stop(brake);
  chassis.drivetrainNeedsStopped = false;
  showMotorTemperature();
  double t1 = Brain.Timer.time(sec);
  printControllerScreen("Running test...");
  // Write test code here:

  chassis.setHeading(0);



  double t2 = Brain.Timer.time(sec);
  char timeMsg[30];
  sprintf(timeMsg, "run  time: %.1f", t2-t1);
  printControllerScreen(timeMsg);
  chassis.stop(hold);
  wait(500, msec);
  chassis.stop(coast); 
 // showMotorTemperature();
  additionalSetupDriver();
}
