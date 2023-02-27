package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.RotatorSubsystem;

public class RotatorToPosition extends CommandBase {
    private final RotatorSubsystem rotatorSubsystem;
    private final double setpoint;
    public RotatorToPosition(RotatorSubsystem rotatorSubsystem, double setpoint)
    {
        this.rotatorSubsystem = rotatorSubsystem;
        this.setpoint = setpoint;
        addRequirements(rotatorSubsystem);
    }
    @Override
    public void initialize() {
        rotatorSubsystem.setSetpoint(setpoint);
    }
    @Override
    public boolean isFinished() {
        return rotatorSubsystem.getMeasurement() == setpoint;
    }
    @Override
    public void end(boolean interrupted) {
        rotatorSubsystem.setSetpoint(rotatorSubsystem.getMeasurement());
    }
}
