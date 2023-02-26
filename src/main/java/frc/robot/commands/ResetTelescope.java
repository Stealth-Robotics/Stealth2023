package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TelescopeSubsystem;

public class ResetTelescope extends CommandBase {
    final TelescopeSubsystem tlescope;

    public ResetTelescope(TelescopeSubsystem extender) {
        this.tlescope = extender;
        addRequirements(extender);
    }

    @Override
    public void initialize() {
        tlescope.setElevatorDown_Slowly_();
    }

    @Override
    public void end(boolean interrupted) {
        tlescope.completeReset();
    }

    @Override
    public boolean isFinished() {
        return tlescope.checkVelocity();
    }
}