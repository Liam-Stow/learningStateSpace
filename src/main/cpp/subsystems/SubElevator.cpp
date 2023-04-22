#include "subsystems/SubElevator.h"
#include <frc2/command/Commands.h>

SubElevator::SubElevator()
{
  _motor.SetConversionFactor(PULLEY_RADIUS.value() / GEARING);
  _follower.SetConversionFactor(PULLEY_RADIUS.value() / GEARING);
  _follower.Follow(_motor);
  _motor.SetPIDFF(1, 0, 0, 1);
  _motor.ConfigSmartMotion(2_mps, 5_mps_sq, 0.5_cm);

  frc::SmartDashboard::PutData("Elevator/motor", (wpi::Sendable *)&_motor);
  frc::SmartDashboard::PutBoolean("Elevator/use state space", false);

  frc::SmartDashboard::PutNumber("Elevator/kS", _feedforward.kS.value());
  frc::SmartDashboard::PutNumber("Elevator/kG", _feedforward.kG.value());
  frc::SmartDashboard::PutNumber("Elevator/kV", _feedforward.kV.value());
  frc::SmartDashboard::PutNumber("Elevator/kA", _feedforward.kA.value());
};

frc2::CommandPtr SubElevator::GoToHeight(units::meter_t height)
{
  return frc2::cmd::Either(
      Run([&, height]
          {
            _controllerLoop.SetNextR(frc::Vectord<2>{height.value(), 0}); // go to height and end with zero velocity
            _observer.Correct(frc::Vectord<1>{0}, frc::Vectord<1>{_motor.GetPosition().value()});
            _controllerLoop.Predict(20_ms);
            _motor.SetVoltage(_controllerLoop.U(0) * 1_V); }),

      Run([&, height]
          {
            auto ff = _feedforward.Calculate(_motor.GetVelocityTarget());
           _motor.SetSmartMotionTarget(height, ff); }),
      []
      { return frc::SmartDashboard::GetBoolean("Elevator/use state space", false); });
}

void SubElevator::Periodic()
{
  _feedforward.kS = frc::SmartDashboard::GetNumber("Elevator/kS", _feedforward.kS.value()) * 1_V;
  _feedforward.kG = frc::SmartDashboard::GetNumber("Elevator/kG", _feedforward.kG.value()) * 1_V;
  _feedforward.kV = frc::SmartDashboard::GetNumber("Elevator/kV", _feedforward.kV.value()) * 1_V/1_mps;
  _feedforward.kA = frc::SmartDashboard::GetNumber("Elevator/kA", _feedforward.kA.value()) * 1_V/1_mps_sq;
}

void SubElevator::SimulationPeriodic()
{
  _sim.SetInputVoltage(_motor.GetSimVoltage());
  _sim.Update(20_ms);
  _motor.UpdateSimEncoder(_sim.GetPosition(), _sim.GetVelocity());
}