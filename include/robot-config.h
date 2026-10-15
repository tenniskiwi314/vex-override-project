#pragma once
using namespace vex;

#define bot_4motor_direct_4inch // you can define multiple robots with different motor configs

#ifdef one_motor_test
const int LEFT_PORT1 = -21;
const int LEFT_PORT2 = -21;
const int LEFT_PORT3 = -21;  // use unused port for the third left motor
const int RIGHT_PORT1 = -21;
const int RIGHT_PORT2 = -21;
const int RIGHT_PORT3 = -21; // use unused port for the third right motor
const float WHEEL_DIAMETER = 4.0;
const float GEAR_RATIO = 1/3.0;  // default cartiridge is 600rpm

const int DRIVE_MODE = 1;  //1 = double arcade, 2 = tank
const bool LEFT_RIGHT_STICK_SWAP = false;

const int INERTIAL_PORT = -21;  //use unused negative port number to skip inertial sensor setup
const int INERTIAL_SCALE = 360;

const int GRABBER_PORT = 1;  // use unused port for the grabber motor

const int LIFT_PORT1 = 11;
const int LIFT_PORT2 = -12;  // use unused port for the second lift motor
const int LIFT_SENSOR_PORT = -21;  // use unused port for the lift rotation sensor

const int ROLLER_PORT = -21;  // use unused port for the roller motor

const int NUMBER_OF_MOTORS = 1;
const float LIFT_MAX_HEIGHT = 0;  // the max lift position is not set

#endif

#ifdef bot_4motor_direct_4inch
const int LEFT_PORT1 = 16;
const int LEFT_PORT2 = 4;
const int LEFT_PORT3 = -21;  // use unused port for the third left motor
const int RIGHT_PORT1 = -18;
const int RIGHT_PORT2 = -10;
const int RIGHT_PORT3 = -21; // use unused port for the third right motor
const float WHEEL_DIAMETER = 4.0;
const float GEAR_RATIO = 1/3.0;  // default cartridge is 600rpm

const int DRIVE_MODE = 1;  //1 = double arcade, 2 = tank
const bool LEFT_RIGHT_STICK_SWAP = false;

const int INERTIAL_PORT = 1;  //use unused negative port number to skip inertial sensor setup
const int INERTIAL_SCALE = 360;

const int GRABBER_PORT = -21;  // use unused port for the grabber motor
const int LIFT_PORT1 = -21;  // use unused port for the first lift motor 
const int LIFT_PORT2 = -21;  // use unused port for the second lift motor
const int ROLLER_PORT = -21;  // use unused port for the roller motor

const int NUMBER_OF_MOTORS = 4;
const float LIFT_MAX_HEIGHT = 0;  // the max lift position is not set
const int LIFT_SENSOR_PORT = -21;  // use unused port for the lift rotation sensor

#endif

// external variables
extern motor_group liftMotor;
extern motor grabberMotor;
extern motor rollerMotor;

float speedLimitWithHeight();

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

