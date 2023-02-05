#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc/system/plant/LinearSystemId.h>
#include <frc/system/plant/DCMotor.h>
#include <frc/simulation/ElevatorSim.h>
#include <frc/controller/ElevatorFeedforward.h>
#include <units/voltage.h>
#include <units/angular_velocity.h>
#include <units/angular_acceleration.h>
#include <units/moment_of_inertia.h>
#include <frc/estimator/KalmanFilter.h>
#include <frc/controller/LinearQuadraticRegulator.h>
#include <frc/system/LinearSystemLoop.h>
#include <frc2/command/CommandPtr.h>

#include "utilities/ICSparkMax.h"

class SubElevator : public frc2::SubsystemBase
{
public:
  SubElevator();

  void Periodic() override;
  void SimulationPeriodic() override;

  frc2::CommandPtr GoToHeight(units::meter_t height);

private:
  ICSparkMax<units::meters> _motor{3};
  ICSparkMax<units::meters> _follower{4};

  static constexpr auto MOTORS = frc::DCMotor::NEO(2);
  static constexpr auto MASS = 5_kg;
  static constexpr units::meter_t PULLEY_RADIUS = 2_cm;
  static constexpr auto GEARING = 7.0;

  // state space contol objects
  frc::LinearSystem<2, 1, 1> _plant = frc::LinearSystemId::ElevatorSystem(MOTORS, MASS, PULLEY_RADIUS, GEARING);
  frc::KalmanFilter<2, 1, 1> _observer{_plant, {0.05, 0.5}, {0.0001}, 20_ms};
  frc::LinearQuadraticRegulator<2, 1> _feedback{_plant, {0.02, 0.2}, {10}, 20_ms};
  frc::LinearSystemLoop<2, 1, 1> _controllerLoop{_plant, _feedback, _observer, 12_V, 20_ms};

  // Motion profile + PIDFF objects
  frc::ElevatorFeedforward _feedforward{0_V, 0.32_V, 7_V / 1_mps, 0_V / 1_mps_sq};

  // sim
  frc::sim::ElevatorSim _sim{MOTORS, GEARING, MASS, PULLEY_RADIUS, 0_m, 2_m, true, {0.0001}};
};
