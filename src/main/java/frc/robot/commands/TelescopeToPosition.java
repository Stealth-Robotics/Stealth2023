package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TelescopeSubsystem;

public class TelescopeToPosition extends CommandBase {
    // The subsystem the command runs on
    private final TelescopeSubsystem telescopeSubsystem;
    // Setpoint to run the telescope to
    // In percent of maximum extension
    private final double percent;
    
    public TelescopeToPosition(TelescopeSubsystem telescopeSubsystem, double percent) {
        this.telescopeSubsystem = telescopeSubsystem;
        percent = MathUtil.clamp(percent, 0.025, 0.9);
        this.percent = percent;
    }

    @Override
    public void initialize() {
        // Set the setpoint to the desired position
        telescopeSubsystem.setExtensionPercent(percent);
    }

    @Override
    public void end(boolean interrupted) {
        // Run the telescope to the setpoint
        if (interrupted) telescopeSubsystem.setToCurrentPosition();
    }

    @Override
    public boolean isFinished() {
        // To end, check if we are at the setpoint
        return telescopeSubsystem.atSetpoint();
    }
}
