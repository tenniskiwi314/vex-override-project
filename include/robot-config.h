#pragma once
using namespace vex;

// you can define multiple robots with different motor configs
#define practice_bot 

#define UNUSED_PORT -21

#ifdef practice_bot
const int LEFT_PORT1 = 1;
const int LEFT_PORT2 = -2;
const int LEFT_PORT3 = UNUSED_PORT;
const int RIGHT_PORT1 = 9;
const int RIGHT_PORT2 = -10;
const int RIGHT_PORT3 = UNUSED_PORT;
const float WHEEL_DIAMETER = 2.75;
const float GEAR_RATIO = 0.75;

const int DRIVE_MODE = 0; //1 = double arcade, 2 = tank, 0 = single arcade
const bool LEFT_RIGHT_STICK_SWAP = false;
const int SPEED_LIMIT = 100; // drive speed limit when lift is at max height, set to 100 if no speed reduction is needed

const int INERTIAL_PORT = 8;
const int INERTIAL_SCALE = 360;

const int GRABBER_PORT = -3;

const int LIFT_PORT1 = 4;
const int LIFT_PORT2 = UNUSED_PORT;
const int LIFT_SENSOR_PORT = 5;  
const float LIFT_MAX_HEIGHT = 600; // the max lift position, Set to 0 if lift position can not be correctly measured.

const int ROLLER_PORT = 11;  
const int OPTICAL_PORT = UNUSED_PORT;  

const int NUMBER_OF_MOTORS = 7;

#endif

#ifdef one_motor_test
const int LEFT_PORT1 = UNUSED_PORT;
const int LEFT_PORT2 = UNUSED_PORT;
const int LEFT_PORT3 = UNUSED_PORT;  // use unused port for the third left motor
const int RIGHT_PORT1 = UNUSED_PORT;
const int RIGHT_PORT2 = UNUSED_PORT;
const int RIGHT_PORT3 = UNUSED_PORT; // use unused port for the third right motor
const float WHEEL_DIAMETER = 4.0;
const float GEAR_RATIO = 1/3.0;  // default cartiridge is 600rpm

const int DRIVE_MODE = 1;  //1 = double arcade, 2 = tank
const bool LEFT_RIGHT_STICK_SWAP = false;
const int SPEED_LIMIT = 100; // drive speed limit when lift is at max height, set to 100 if no speed reduction is needed

const int INERTIAL_PORT = UNUSED_PORT;  //use unused negative port number to skip inertial sensor setup
const int INERTIAL_SCALE = 360;

const int GRABBER_PORT = UNUSED_PORT;  // use unused port for the grabber motor

const int LIFT_PORT1 = 1;
const int LIFT_PORT2 = -2;  // use unused port for the second lift motor
const int LIFT_SENSOR_PORT = UNUSED_PORT;  // use unused port for the lift rotation sensor
const float LIFT_MAX_HEIGHT = 0;  // the max lift position is not set

const int ROLLER_PORT = UNUSED_PORT;  
const int OPTICAL_PORT = UNUSED_PORT;  // use unused port for the optical sensor

const int NUMBER_OF_MOTORS = 2;

#endif

// external variables
extern motor_group liftMotor;
extern motor grabberMotor;
extern motor rollerMotor;

float speedLimitWithHeight();
float getLiftPosition(bool printPosition = false);

// ------------------------------------------------------------------------
//              System setup
// ------------------------------------------------------------------------

// A global instance of the brain used for printing to the V5 Brain screen.
extern brain Brain;
extern controller controller1;
// Forward declaration of the Drive class.
class Drivetrain;
// A global instance of the Drive class.
extern Drivetrain chassis;
extern int drive_mode;

void additionalSetupDriver();
void additionalSetupAuton();
void setChassisDefaults();
void quickTestCode();
void changeDriveMode();
void usercontrol();
void setupButtonMapping();

