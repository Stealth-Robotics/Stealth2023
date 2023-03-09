package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TelescopeSubsystem;

public class TelescopeToPosition extends CommandBase {
    // The subsystem the command runs on
    private final TelescopeSubsystem telescopeSubsystem;
    //Setpoint to run the telescope to
    //In percent of maximum extension
    private final double percent;

    public TelescopeToPosition(TelescopeSubsystem telescopeSubsystem, double percent)
    {
        this.telescopeSubsystem = telescopeSubsystem;
        this.percent = percent;
    }
    
    @Override
    public void initialize() {
        //Set the setpoint to the desired position
        telescopeSubsystem.setSetpoint(percent);
    }

    @Override
    public void execute() {
        //Run the telescope to the setpoint
        telescopeSubsystem.setPositionPercent(telescopeSubsystem.getSetpoint());
    }

    @Override
    public boolean isFinished() {
        //To end, check if we are at the setpoint
        return telescopeSubsystem.atSetpoint();
    }
}
