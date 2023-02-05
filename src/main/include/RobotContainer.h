// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/button/CommandGenericHID.h>
#include "subsystems/SubShooter.h"
#include "subsystems/SubElevator.h"

class RobotContainer {
 public:
  RobotContainer();

  frc2::CommandPtr GetAutonomousCommand();

 private:
  void ConfigureBindings();
  frc2::CommandGenericHID _controller{0};

  SubShooter _shooter;
  SubElevator _elevator;
};
