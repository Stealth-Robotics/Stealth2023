package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.EndEffectorSubsystem;

public class RunEndEffectorMotors extends CommandBase{
    private final EndEffectorSubsystem subsystem;
    private final double speed;
    public RunEndEffectorMotors(EndEffectorSubsystem subsystem, double speed) {
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
