#include "subsystems/SubShooter.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/Commands.h>

SubShooter::SubShooter()
{
    _flywheelMotor.SetConversionFactor(2 * 3.14159);
    _flywheelMotorFollower.SetConversionFactor(2 * 3.14159);

    _flywheelMotorFollower.Follow(_flywheelMotor);
    frc::SmartDashboard::PutData("Shooter/motor", (wpi::Sendable *)&_flywheelMotor);

    frc::SmartDashboard::PutBoolean("Shooter/use state space", false);
}

void SubShooter::Periodic()
{
}

frc2::CommandPtr SubShooter::SpinUpTo(units::radians_per_second_t targetVel)
{
    return frc2::cmd::Either(
        Run([&, targetVel]
            {
                _flywheelLoop.SetNextR(frc::Vectord<1>{targetVel.value()});
                _flywheelLoop.Correct(frc::Vectord<1>{_flywheelMotor.GetVelocity().value()});
                _flywheelLoop.Predict(LOOP_TIME);
                _flywheelMotor.SetVoltage(_flywheelLoop.U(0) * 1_V);
            }),
        Run([&, targetVel]
            { _flywheelMotor.SetVelocityTarget(targetVel); }),
        []
        { return frc::SmartDashboard::GetBoolean("Shooter/use state space", false); });
}

void SubShooter::SimulationPeriodic()
{
    _flywheelSim.SetInputVoltage(_flywheelMotor.GetSimVoltage());
    _flywheelSim.Update(LOOP_TIME);
    _flywheelMotor.UpdateSimEncoder(0_deg, _flywheelSim.GetAngularVelocity());
}