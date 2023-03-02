package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.CrocodileSubsystem;

public class RunCrocodileMotors extends CommandBase{
    private final CrocodileSubsystem subsystem;
    private final double speed;
    public RunCrocodileMotors(CrocodileSubsystem subsystem, double speed) {
        this.subsystem = subsystem;
        this.speed = speed;
        addRequirements(subsystem);
    }
    @Override
    public void initialize() {
        subsystem.setMotorSpeed(speed);
    }
    @Override
    public void end(boolean interrupted) {
        subsystem.setMotorSpeed(0);
    }
    
}
