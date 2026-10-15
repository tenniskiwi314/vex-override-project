#pragma once
#include "vex.h"
#include <string>

// A class to control the robot's drivetrain.
class Drivetrain
{
private:

  // The motor group for the left side of the drivetrain.
  motor_group leftDrive;
  // The motor group for the right side of the drivetrain.
  motor_group rightDrive;

  // The diameter of the wheels.
  float wheelDiameter = 2.75;
  // The gear ratio of the drivetrain. motor/wheel ratio.
  float gearRatio = 0.75;

  // max voltages for driving, turning, and heading correction.
  float driveMaxVoltage = 10, headingMaxVoltage = 6, turnMaxVoltage = 10;

  // PID constants for turning.
  float turnKp = 0.2, turnKi = 0.015, turnKd = 1.5, turnStarti = 7.5;

  // turn exit conditions.
  float turnSettleError = 1.5, turnSettleTime = 300, turnTimeout = 3000;

  // PID constants for driving.
  float driveKp = 1.5, driveKi = 0, driveKd = 10, driveStarti = 0;

  // drive exit conditions.
  float driveSettleError = 1, driveSettleTime = 300, driveTimeout = 3000;

  // PID constants for maintaining heading while driving.
  float headingKp = 0.4, headingKd = 1;

  // Constants for arcade drive.
  float kBrake = 0.5, kTurnBias = 0.5, kTurnDampingFactor = 0.85;

  // allows for a non-proportional steering response
  float kThrottle = 5, kTurn = 10;

  // The brake type for the drivetrain.
  vex::brakeType stopMode = coast;

  // Gyro scale factor
  float gyro_scale = 360;

  // Denoise factor for distance sensors
  float denoise = 0.4;

  // The target heading of the robot.
  float targetHeading;

  // Gets the position of the left side of the drivetrain in inches.
  float getLeftPosition();
  // Gets the position of the right side of the drivetrain in inches.
  float getRightPosition();

public: 
  // The inertial sensor.
  inertial inertialSensor;

  // optional: use distance sensors for driving API
  distance frontSensor = distance(-1); 
  distance backSensor = distance(-1);  

  // The constructor for the Drive class.
  Drivetrain(motor_group leftDrive, motor_group rightDrive, inertial inertialSensor);

  // Sets the current heading of the robot.
  void setHeading(float orientationDeg);
  // Drives the robot with a specific voltage for each side of the drivetrain.
  void driveWithVoltage(float leftVoltage, float rightVoltage);

  // Turns the robot to a specific heading.
  void turn(float heading);
  // turn slower with lower max voltage
  void turn(float heading, float turnMaxVoltage); 
  // earlyExitFactor for motion chaining: nonstopping if greater than 1. Maxium is 10.
  void turn(float heading, float turnMaxVoltage, float earlyExitFactor); 
  // only one side moves. swing = 1: right side swings, swing = -1: left side swings.
  void turn(float heading, float turnMaxVoltage, float earlyExitFactor, int swing);

  // Drives the robot a specific distance.
  void drive(float distance);
  // Drives the robot a specific distance with a maximum voltage.
  void drive(float distance, float driveMaxVoltage);
  // earlyExitFactor for motion chaining: nonstopping if greater than 1. Maxium is 10.
  void drive(float distance, float heading, float earlyExitFactor);
  // distance > 0: front sensor, distance < 0: back sensor
  void drive(float distance, float heading, float earlyExitFactor, bool distanceSensing); 
  // Curve drive with max voltage and heading correction
  void drive(float distance, float driveMaxVoltage, float heading, float headingMaxVoltage,  float earlyExitFactor);
  void drive(float distance, float driveMaxVoltage, float heading, float headingMaxVoltage,  float earlyExitFactor, bool distanceSensing);

    // Stops the drivetrain.
  void stop(vex::brakeType mode);

  // Gets the current heading of the robot.
  float getHeading();

  // A flag to indicate if the drivetrain needs to be stopped.
  bool drivetrainNeedsStopped = false;

  // Controls the robot in arcade mode.
  void controlArcade(int throttle, int turn);
  // Controls the robot in tank mode.
  void controlTank(int left, int right);

  // sets the wheel settings
  void setWheelSettings(float wheelDiameter, float gearRatio);

  // Sets the PID constants for driving.
  void setDrivePID(float driveKp, float driveKi, float driveKd, float driveStarti);
  // Sets the exit conditions for driving.
  void setDriveExitConditions(float driveSettleError, float driveSettleTime, float driveTimeout);
  // Sets the PID constants for maintaining heading.
  void setHeadingPID(float headingKp, float headingKd);
  // Sets the exit conditions for turning.
  void setTurnExitConditions(float turnSettleError, float turnSettleTime, float turnTimeout);
  // Sets the PID constants for turning.
  void setTurnPID(float turnKp, float turnKi, float turnKd, float turnStarti); 
  void setMaxVoltage(float turnMaxVoltage, float driveMaxVoltage, float headingMaxVoltage);
  
  // Sets the constants for arcade drive.
  void setArcadeConstants(float kBrake, float kTurnBias, float kTurnDampingFactor);

  // Sets the gyro scale factor
  void setGyroScale(float gyroScale);

  // Sets distance sensors
  void setDistanceSensors(int direction, distance distSensor, float denoise = 0.4);

  void controlMecanum(int x, int y, int acc, int steer, motor DriveLF, motor DriveLR, motor DriveRF, motor DriveRB);

};