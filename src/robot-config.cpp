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
    float speedLimit = speedLimitWithHeight();
    switch (drive_mode) {  
    case 1: // double arcade
      throttle = threshold(controller1.Axis2.position(), -speedLimit, speedLimit);
      turn = controller1.Axis4.position();
      if (LEFT_RIGHT_STICK_SWAP) {
        throttle = threshold(controller1.Axis3.position(), -speedLimit, speedLimit);
        turn = threshold(controller1.Axis1.position(), -sqrt(speedLimit)*10, sqrt(speedLimit)*10);
      }
      chassis.controlArcade(throttle, turn);
      break;
    case 0: // single arcade
      throttle = threshold(controller1.Axis2.position(), -speedLimit, speedLimit);
      turn = controller1.Axis1.position();
      if (LEFT_RIGHT_STICK_SWAP) {
        throttle = threshold(controller1.Axis3.position(), -speedLimit, speedLimit);
        turn = controller1.Axis4.position();
      }
      chassis.controlArcade(throttle, turn);
      break;
    case 2: // tank drive
      leftSpeed = threshold(controller1.Axis3.position(), -speedLimit, speedLimit);
      rightSpeed = threshold(controller1.Axis2.position(), -speedLimit, speedLimit);
      chassis.controlTank(leftSpeed, rightSpeed);    
      break;
    }
    
    Brain.Screen.setCursor(10, 1);
    Brain.Screen.print("Heading : %.1f", chassis.getHeading());

    // This wait prevents the loop from using too much CPU time.
    wait(20, msec);
  } 
}


// ------------------------------------------------------------------------
//        Other subsystems: motors, & sensors
// ------------------------------------------------------------------------
motor grabberMotor = motor(numberToPort(GRABBER_PORT), ratio18_1, port_reverse(GRABBER_PORT));
motor liftMotor1 = motor(numberToPort(LIFT_PORT1), ratio36_1, port_reverse(LIFT_PORT1));
motor liftMotor2 = motor(numberToPort(LIFT_PORT2), ratio36_1, port_reverse(LIFT_PORT2));
motor_group liftMotor = motor_group(liftMotor1, liftMotor2);
rotation liftSensor = rotation(numberToPort(LIFT_SENSOR_PORT), port_reverse(LIFT_SENSOR_PORT));

motor rollerMotor = motor(numberToPort(ROLLER_PORT), ratio18_1, port_reverse(ROLLER_PORT));


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
 if (liftSensor.installed()) {
   liftSensor.setPosition(0, degrees);
 }
}

// ------------------------------------------------------------------------
//        Helper functions definition
// ------------------------------------------------------------------------

void moveLift(int velocityPercent)
{
  liftMotor.spin(forward, velocityPercent, percent);
  if (velocityPercent == 0) 
  {
    liftMotor.stop(hold);
  }
}

float getLiftPosition(bool printPosition = false) {
  float p = liftMotor.position(degrees);
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
    return 100.0 - (height * 50.0 / LIFT_MAX_HEIGHT);
  } else {
    return 50;
  } 
}

bool liftWithinLimit()
{
  if (LIFT_MAX_HEIGHT <= 0) return true; // if no max lift position 
  float height = getLiftPosition();

  if (height > LIFT_MAX_HEIGHT) {
    liftMotor.stop(hold);
    return false;
  }
  return true;
}

bool liftAboveGround()
{
  if (LIFT_MAX_HEIGHT <= 0) return true; // no ground position 
  float height = getLiftPosition();
  if (height > 0) {
    return true;
  }
  liftMotor.stop(coast);
  return false;
}

void raiseLiftABit()
{
  moveLift(30);
  wait(150, msec);
  moveLift(0);
}

void dropLiftoGround()
{
  while(liftAboveGround()){
    liftMotor.spin(forward, -8, volt); // let the weight pull the lift down
    wait(30, msec);
  } 
}

bool autoGrab()
{
//  return false;
  float c = grabberMotor.current(amp);
  if (c > 2.5) // intake motor is stalled means cup is grabbed, stop intake and hold cup
  {
    return true;
  }
  return false;
}


// ------------------------------------------------------------------------
//              Button controls
// ------------------------------------------------------------------------

void buttonUpFunction() {
  moveLift(80);
  wait(600, msec);
  while(controller1.ButtonUp.pressing()) { 
    moveLift(80);
    wait(20, msec); 
  }
  moveLift(0);
  getLiftPosition(true);
}

void buttonDownFunction() {
  if (controller1.ButtonR2.pressing())  // reset lift sensor
  {
    controller1.rumble("-");
    printControllerScreen("Resetting lift sensor");
    liftSensor.setPosition(0, degrees); 
  }

  moveLift(-20);
  wait(600, msec);
  while(controller1.ButtonDown.pressing()) { 
    moveLift(-30);
    wait(20, msec); 
  }
  moveLift(0);
  getLiftPosition(true);
}

void buttonLeftFunction() {
  // Implement left button action here
  grabberMotor.spin(forward, 5, volt);
  wait(600, msec);
  while(controller1.ButtonLeft.pressing()) { 
    grabberMotor.spin(forward, 10, volt);
    wait(20, msec); 
  }
  grabberMotor.stop(coast);
}

void buttonRightFunction() {
  grabberMotor.spin(reverse, 5, volt);
  wait(600, msec);
  while(controller1.ButtonRight.pressing()) { 
    grabberMotor.spin(reverse, 10, volt);
    wait(20, msec); 
  }
  grabberMotor.stop(coast);
}

void buttonL1Function() {
  
    grabberMotor.spin(forward, 10, volt);
    wait(500, msec);
    grabberMotor.stop(coast);
  
    while(controller1.ButtonL1.pressing()) {
      wait(20, msec);
    }

  grabberMotor.spin(reverse, 10, volt);
  wait(500, msec);
  grabberMotor.stop(coast);
}

void buttonR1Function() {
  while (controller1.ButtonR1.pressing()) {
    rollerMotor.spin(reverse, 10, volt);
  }
  rollerMotor.stop(coast); 
}

// toggle drivetrain brake, e.g. when protecting the roller
bool drivetrainBrakeOn = false;
void buttonBFunction (){
  controller1.rumble("-");
  printControllerScreen("Toggling brake");
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
  controller1.ButtonLeft.pressed(buttonLeftFunction);
  controller1.ButtonRight.pressed(buttonRightFunction);
  controller1.ButtonL1.pressed(buttonL1Function);
  controller1.ButtonA.pressed(quickTestCode);
  controller1.ButtonB.pressed(buttonBFunction);
  
  controller1.ButtonR1.pressed(buttonR1Function);
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
