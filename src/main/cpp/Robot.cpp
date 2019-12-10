/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Robot.h"

#include <iostream>

#include <frc/Timer.h>
#include <frc/smartdashboard/SmartDashboard.h>

Robot::Robot() : drive(), controls(&drive){
 
}

void Robot::RobotInit() {
  
}

void Robot::Autonomous() {
  
}

/**
 * Runs the motors with arcade steering.
 */
void Robot::OperatorControl() {
 
  while (IsOperatorControl() && IsEnabled()) {
    
    controls.process(); 
    
    // The motors will be updated every 5ms
    frc::Wait(0.005);
  }
}


void Robot::Test() {}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
