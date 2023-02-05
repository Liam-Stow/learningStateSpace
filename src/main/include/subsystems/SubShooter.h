#pragma once

#include "utilities/ICSparkMax.h"

#include <frc2/command/SubsystemBase.h>
#include <frc/system/plant/LinearSystemId.h>
#include <frc/system/plant/DCMotor.h>
#include <frc/simulation/FlywheelSim.h>
#include <units/voltage.h>
#include <units/angular_velocity.h>
#include <units/angular_acceleration.h>
#include <units/moment_of_inertia.h>
#include <frc/estimator/KalmanFilter.h>
#include <frc/controller/LinearQuadraticRegulator.h>
#include <frc/system/LinearSystemLoop.h>
#include <frc2/command/CommandPtr.h>

class SubShooter : public frc2::SubsystemBase
{
public:
  SubShooter();

  void Periodic() override;
  void SimulationPeriodic() override;
  frc2::CommandPtr SpinUpTo(units::radians_per_second_t targetVel);

private:
  ICSparkMax<units::radians> _flywheelMotor{1};
  ICSparkMax<units::radians> _flywheelMotorFollower{2};

  static constexpr auto FLYWHEEL_MOTORS = frc::DCMotor::NEO(2);
  static constexpr auto FLYWHEEL_GEARING = 1.0;
  static constexpr auto FLYWHEEL_MOI = 0.000032_kg_sq_m;

  static constexpr auto LOOP_TIME = 20_ms;

  frc::LinearSystem<1, 1, 1> _flywheelPlant = frc::LinearSystemId::FlywheelSystem(FLYWHEEL_MOTORS, FLYWHEEL_MOI, FLYWHEEL_GEARING);
  frc::KalmanFilter<1, 1, 1> _flywheelObserver{_flywheelPlant, {3.0}, {0.01}, LOOP_TIME};
  frc::LinearQuadraticRegulator<1, 1> _feedbackController{_flywheelPlant, {8.0}, {10.0}, LOOP_TIME};
  frc::LinearSystemLoop<1,1,1> _flywheelLoop{_flywheelPlant, _feedbackController, _flywheelObserver, 12_V, LOOP_TIME};

  frc::sim::FlywheelSim _flywheelSim{FLYWHEEL_MOTORS, FLYWHEEL_GEARING, FLYWHEEL_MOI};
};
