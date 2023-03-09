package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TelescopeSubsystem;

public class ResetTelescope extends CommandBase {
    final TelescopeSubsystem tlescope;

    public ResetTelescope(TelescopeSubsystem extender) {
        this.tlescope = extender;
        addRequirements(extender);
    }

    // Starts retracting the telescope
    @Override
    public void initialize() {
        tlescope.retractTelescope();
    }

    // Stops the telescope and resets the encoder
    @Override
    public void end(boolean interrupted) {
        tlescope.completeReset();
    }

    // If our debouncer says we are at the bottom, we are done
    @Override
    public boolean isFinished() {
        return tlescope.checkVelocity();
    }
}