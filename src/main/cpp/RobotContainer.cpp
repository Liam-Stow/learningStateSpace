// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc2/command/Commands.h>

RobotContainer::RobotContainer() {
  ConfigureBindings();
}

void RobotContainer::ConfigureBindings() {
  using namespace frc2::cmd;
  _controller.Button(1).OnTrue(_shooter.SpinUpTo(3000_rpm));
  _controller.Button(2).OnTrue(_shooter.SpinUpTo(0_rpm));
  _controller.Button(3).OnTrue(_elevator.GoToHeight(1.5_m));
  _controller.Button(4).OnTrue(_elevator.GoToHeight(0_m));
}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
  return frc2::cmd::Print("No autonomous command configured");
}
