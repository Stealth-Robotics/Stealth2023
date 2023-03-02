package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TelescopeSubsystem;

public class TelescopeToPosition extends CommandBase {

    private final TelescopeSubsystem telescopeSubsystem;
    private final double setpoint;

    public TelescopeToPosition(TelescopeSubsystem telescopeSubsystem, double ticks)
    {
        this.telescopeSubsystem = telescopeSubsystem;
        this.setpoint = ticks;
    }
    
    @Override
    public void initialize() {
        telescopeSubsystem.setSetpoint(setpoint);
    }

    @Override
    public void execute() {
        telescopeSubsystem.setPosition(telescopeSubsystem.getSetpoint());
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return telescopeSubsystem.atSetpoint();
    }
}
