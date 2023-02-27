package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.EndEffectorSubsystem;

public class EndEffectorDefaultCommand extends CommandBase {
    private final EndEffectorSubsystem subsystem;
    private final DoubleSupplier motorSpeed;

    public EndEffectorDefaultCommand(EndEffectorSubsystem subsystem,DoubleSupplier motorSpeed) {
        this.subsystem = subsystem;
        this.motorSpeed = motorSpeed;
        addRequirements(subsystem);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        subsystem.setMotorSpeed(motorSpeed.getAsDouble());
    }
}
