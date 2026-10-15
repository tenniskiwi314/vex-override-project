#include "vex.h"

Drivetrain::Drivetrain(motor_group leftDrive, motor_group rightDrive, inertial inertialSensor):
  leftDrive(leftDrive),
  rightDrive(rightDrive),
  inertialSensor(inertialSensor) {}

// ------------------------------------------------------------------------
//               additional drivetain parameter configuration
// ------------------------------------------------------------------------

void Drivetrain::setWheelSettings(float wheelDiameter, float gearRatio)
{
  this -> wheelDiameter = wheelDiameter;
  this -> gearRatio = gearRatio;
}

  //acts like constructor but work like this for readability. parameters for this and below has default values
void Drivetrain::setMaxVoltage(float turnMaxVoltage, float driveMaxVoltage, float headingMaxVoltage)
{ 
  this -> driveMaxVoltage = driveMaxVoltage;
  this -> turnMaxVoltage = turnMaxVoltage;
  this -> headingMaxVoltage = headingMaxVoltage;
}

void Drivetrain::setTurnPID(float turnKp, float turnKi, float turnKd, float turnStarti) {
  this -> turnKp = turnKp;
  this -> turnKi = turnKi;
  this -> turnKd = turnKd;
  this -> turnStarti = turnStarti;
}

void Drivetrain::setDrivePID(float driveKp, float driveKi, float driveKd, float driveStarti) {
  this -> driveKp = driveKp;
  this -> driveKi = driveKi;
  this -> driveKd = driveKd;
  this -> driveStarti = driveStarti;
}

void Drivetrain::setHeadingPID(float headingKp, float headingKd) {
  this -> headingKp = headingKp;
  this -> headingKd = headingKd;
}

void Drivetrain::setTurnExitConditions(float turnSettleError, float turnSettleTime, float turnTimeout) {
  this -> turnSettleError = turnSettleError;
  this -> turnSettleTime = turnSettleTime;
  this -> turnTimeout = turnTimeout;
}

void Drivetrain::setDriveExitConditions(float driveSettleError, float driveSettleTime, float driveTimeout) {
  this -> driveSettleError = driveSettleError;
  this -> driveSettleTime = driveSettleTime;
  this -> driveTimeout = driveTimeout;
}

void Drivetrain::setGyroScale(float gyroScale)
{
  this -> gyro_scale = gyroScale;
  setHeading(inertialSensor.heading());
}

float Drivetrain::getLeftPosition() {
  return leftDrive.position(deg) / 360.0 * gearRatio  * M_PI * wheelDiameter;
}

float Drivetrain::getRightPosition() {
  return rightDrive.position(deg) / 360.0 * gearRatio * M_PI * wheelDiameter;
}

void Drivetrain::setDistanceSensors(int direction, distance distSensor, float denoise) {
 
  if (!distSensor.installed())
  {
    printControllerScreen("distance sensor failure", true);
    return;
  }
  if (direction == 1) this-> frontSensor = distSensor;
  if (direction == 3) this->backSensor = distSensor;
  this->denoise = denoise;
 
}

void Drivetrain::setArcadeConstants(float kBrake, float kTurnBias, float kTurnDampingFactor)//in config.cpp
{
  this->kBrake = kBrake;
  this->kTurnBias = kTurnBias;
  this->kTurnDampingFactor = kTurnDampingFactor;
}

// ------------------------------------------------------------------------
//              drive APIs for autons
// ------------------------------------------------------------------------

void Drivetrain::setHeading(float orientationDeg) {
  inertialSensor.setRotation(orientationDeg*gyro_scale/360.0, deg);
  targetHeading = orientationDeg*gyro_scale/360.0;//remember target heading so if interupted, still try correct in next step
}

float Drivetrain::getHeading() {
  return( normalize360( inertialSensor.rotation()*360.0/gyro_scale ) ); //normalize functions in util
}

void Drivetrain::driveWithVoltage(float leftVoltage, float rightVoltage) {
  leftDrive.spin(fwd, leftVoltage, volt);
  rightDrive.spin(fwd, rightVoltage, volt);
}

void Drivetrain::turn(float heading) {
  turn(heading, turnMaxVoltage);
}

void Drivetrain::turn(float heading, float turnMaxVoltage) {
  turn(heading, turnMaxVoltage, 1);
}

void Drivetrain::turn(float heading, float turnMaxVoltage, float earlyExitFactor)
{
  turn(heading, turnMaxVoltage, earlyExitFactor, 0);
}

void Drivetrain::turn(float heading, float turnMaxVoltage, float earlyExitFactor, int swing)
{
  if (earlyExitFactor > 10) earlyExitFactor = 10;
  if (earlyExitFactor < 1) earlyExitFactor = 1;
  targetHeading = normalize360(heading);
  float adjustedKp = turnKp;
  if (swing!=0) adjustedKp = adjustedKp * 2;
  PID turnPID(adjustedKp, turnKi, turnKd, turnStarti, turnSettleError*earlyExitFactor , turnSettleTime/earlyExitFactor, turnTimeout);
  while (!turnPID.isDone() && !drivetrainNeedsStopped) {
    float error = normalize180(heading - getHeading());
    float output = turnPID.update(error);
    output = threshold(output, -turnMaxVoltage, turnMaxVoltage);
    if (swing == -1)
    {
      leftDrive.stop(hold);
      rightDrive.spin(fwd, -output, volt);
    }
    else if (swing == 1)
    {
      rightDrive.stop(hold);
      leftDrive.spin(fwd, output, volt);
    }
    else if (swing == 0)
    {
      driveWithVoltage(output, -output);
    }
    wait(10, msec);
  }
  if (earlyExitFactor == 1)
  {
    leftDrive.stop(hold);
    rightDrive.stop(hold);
  }
}

//all simpler drive code use the most complicated one but just assumes stuff for the unused parameters
void Drivetrain::drive(float distance) {
  drive(distance, driveMaxVoltage);
}

void Drivetrain::drive(float distance, float driveMaxVoltage) {
  drive(distance, driveMaxVoltage, targetHeading, headingMaxVoltage, 1);
}

void Drivetrain::drive(float distance, float heading, float earlyExitFactor)
{
  drive(distance, driveMaxVoltage, heading, headingMaxVoltage, earlyExitFactor);
}

void Drivetrain::drive(float distance, float heading, float earlyExitFactor, bool distanceSensing) {
  drive(distance, driveMaxVoltage, heading, headingMaxVoltage, earlyExitFactor, distanceSensing);
}

void Drivetrain::drive(float distance, float driveMaxVoltage, float heading, float headingMaxVoltage,  float earlyExitFactor)
{
  drive(distance, driveMaxVoltage, heading, headingMaxVoltage, earlyExitFactor, false);
}

void Drivetrain::drive(float distance, float driveMaxVoltage, float heading, float headingMaxVoltage,  float earlyExitFactor, bool distanceSensing)
{
  if (earlyExitFactor > 10) earlyExitFactor = 10;
  if (earlyExitFactor < 1) earlyExitFactor = 1;
  targetHeading = normalize360(heading);
  PID drivePID(driveKp, driveKi, driveKd, driveStarti, driveSettleError * earlyExitFactor, driveSettleTime/earlyExitFactor, driveTimeout);
  PID headingPID(headingKp, headingKd);//different than turnpid
  float startAveragePosition = (getLeftPosition() + getRightPosition()) / 2.0;
  float averagePosition = startAveragePosition;//get first position
  float objectDistance = 0, prevObjectDistance = 0;

  // if distance is positive, drive forward use front sensor; if negative, drive back use back sensor
  if (distanceSensing)
  {
    if (distance > 0 && frontSensor.installed()){
      prevObjectDistance = frontSensor.objectDistance(inches);
    }
    else if (distance < 0 && backSensor.installed()){
      prevObjectDistance = -1 * backSensor.objectDistance(inches);
    }
    else {
      printControllerScreen("distance sensor failure", true);
      wait(2, seconds);
      return;
    }
    
  }
  while (drivePID.isDone() == false && !drivetrainNeedsStopped) {
    averagePosition = (getLeftPosition() + getRightPosition()) / 2.0;
    objectDistance = prevObjectDistance;
    if (distanceSensing)
    {
      if (distance > 0 && frontSensor.installed())
        objectDistance = frontSensor.objectDistance(inches);
      else if (backSensor.installed())
        objectDistance = -1 * backSensor.objectDistance(inches);
      if (objectDistance == 0 || fabs(objectDistance) > 144) 
      {
        // invalid reading: stop driving
        objectDistance = distance;
      }
      else 
      {// Low-pass filtered distance reading to smooth data
        objectDistance = (1 - denoise) * objectDistance + denoise* prevObjectDistance;
      }
      prevObjectDistance = objectDistance;

    }
    float driveError;
    if (!distanceSensing)
    {
      driveError = distance - (averagePosition - startAveragePosition);//use motor encoder for error
    }
    else
    {
      driveError = objectDistance - distance; // use distance sensor reading for error
    } 
    float headingError = normalize180(targetHeading - getHeading());//get heading error
    float driveOutput = drivePID.update(driveError);//do calculations
    float headingOutput = headingPID.update(headingError);

    driveOutput = threshold(driveOutput, -driveMaxVoltage, driveMaxVoltage);//sets voltage for drive and turn
    headingOutput = threshold(headingOutput, -headingMaxVoltage, headingMaxVoltage);

    driveWithVoltage(driveOutput + headingOutput, driveOutput - headingOutput);//actully drives the curve
    wait(10, msec);
  }
  
  if (earlyExitFactor == 1)
  {
    leftDrive.stop(hold);
    rightDrive.stop(hold);
  }
}

void Drivetrain::stop(vex::brakeType mode) {
  drivetrainNeedsStopped = true;
  leftDrive.stop(mode);
  rightDrive.stop(mode);
  stopMode = mode;
  leftDrive.resetPosition();
  rightDrive.resetPosition();
  drivetrainNeedsStopped = false;
}


// ------------------------------------------------------------------------
//               Driver Control
// ------------------------------------------------------------------------

void Drivetrain::controlArcade(int y, int x) {//need update
  float throttle = deadband(y, 5);//in util
  float turn = deadband(x, 5) * kTurnDampingFactor;

  turn = curveFunction(turn, kTurn);//in util
  throttle = curveFunction(throttle, kThrottle);

  float leftPower = percentToVolt(throttle + turn);//in util
  float rightPower = percentToVolt(throttle - turn);

  if (kTurnBias > 0) {
    float sum = fabs(throttle) + fabs(turn);
    if (sum > 100) {
      float excess = sum - 100;
      float throttleReduce = excess * (1 - kTurnBias);
      float turnReduce     = excess * kTurnBias;
      throttle -= copysign(throttleReduce, throttle);
      turn     -= copysign(turnReduce, turn);
    }
    leftPower = percentToVolt(throttle + turn);
    rightPower = percentToVolt(throttle - turn);
  }

  if (fabs(throttle) > 0 || fabs(turn) > 0) {
    leftDrive.spin(fwd, leftPower, volt);
    rightDrive.spin(fwd, rightPower, volt);
    drivetrainNeedsStopped = true;
  }
  // When joystick are released, run active brake on drive
  // ajdust the coefficient to the amount of coasting preferred
  else {
    if (drivetrainNeedsStopped) {
      if (stopMode != hold) {
        leftDrive.resetPosition();
        rightDrive.resetPosition();
        wait(20, msec);
        leftDrive.spin(fwd, -leftDrive.position(rev) * kBrake, volt);
        rightDrive.spin(fwd, -rightDrive.position(rev) * kBrake, volt);
      } else {
        leftDrive.stop(hold);
        rightDrive.stop(hold);
      }
      drivetrainNeedsStopped = false;
    }
  }
}

void Drivetrain::controlTank(int left, int right) {
  float leftthrottle = left;
  float rightthrottle = right;

  if (fabs(leftthrottle) > 0 || fabs(rightthrottle) > 0) {
    leftDrive.spin(fwd, percentToVolt(leftthrottle), volt);
    rightDrive.spin(fwd, percentToVolt(rightthrottle), volt);
    drivetrainNeedsStopped = true;
  } else {
    if (drivetrainNeedsStopped) {
      leftDrive.stop(stopMode);
      rightDrive.stop(stopMode);
      drivetrainNeedsStopped = false;
    }
  }
}

void Drivetrain::controlMecanum(int x, int y, int acc, int steer, motor DriveLF, motor DriveRF, motor DriveLB, motor DriveRB) {
  float throttle = deadband(y, 5);
  float strafe = deadband(x, 5);
  float straight = deadband(acc, 5);
  float turn = deadband(steer, 5);
  straight = curveFunction(straight, kThrottle);
  turn = curveFunction(turn, kTurn);

  if (turn == 0 && strafe == 0 && throttle == 0 && straight == 0) {
    if (drivetrainNeedsStopped) {
      leftDrive.stop(stopMode);
      rightDrive.stop(stopMode);
      drivetrainNeedsStopped = false;
      return;
    }
  } 

  if (turn == 0 && straight == 0) {
    DriveLF.spin(fwd, percentToVolt(throttle + turn + strafe), volt);
    DriveRF.spin(fwd, percentToVolt(throttle - turn - strafe), volt);
    DriveLB.spin(fwd, percentToVolt(throttle + turn - strafe), volt);
    DriveRB.spin(fwd, percentToVolt(throttle - turn + strafe), volt);
  }
  //aracade drive
  else
  {
    float leftPower = percentToVolt(throttle + turn);
    float rightPower = percentToVolt(throttle - turn);
    leftDrive.spin(fwd, leftPower, volt);
    rightDrive.spin(fwd, rightPower, volt);
    drivetrainNeedsStopped = true;
  }
}
