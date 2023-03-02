package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.CrocodileSubsystem;

public class CrocodileDefaultCommand extends CommandBase {
    private final CrocodileSubsystem subsystem;
    private final DoubleSupplier motorSpeed;
    private final BooleanSupplier slowMovement;

    public CrocodileDefaultCommand(CrocodileSubsystem subsystem, DoubleSupplier motorSpeed, BooleanSupplier slowMovement) {
        this.subsystem = subsystem;
        this.motorSpeed = motorSpeed;
        this.slowMovement = slowMovement;
        addRequirements(subsystem);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (slowMovement.getAsBoolean()) {
            subsystem.setMotorSpeed(motorSpeed.getAsDouble() * 0.2);
            return;
        }
        subsystem.setMotorSpeed(motorSpeed.getAsDouble());
    }
}
