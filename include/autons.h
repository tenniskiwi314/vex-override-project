#pragma once
#include "rgb-template/drivetrain.h"

void front_auton();
void side_auton();
void skill();

extern int skillStep;
// ------------------------------------------------------------------------
//               standard auton functions and variables
// ------------------------------------------------------------------------
void pre_auton();
void autonomous();
void exitAuton();

void registerAutonTestButtons();

extern int currentAutonSelection;
extern bool autonTestMode;
extern bool endGameTimerEnabled;

extern int teamColor;
void changeTeamColor();
void saveConfigParameters();


