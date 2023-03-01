package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.CrocodileSubsystem;

public class CrocodileDefaultCommand extends CommandBase {
    private final CrocodileSubsystem subsystem;
    private final DoubleSupplier motorSpeed;

    public CrocodileDefaultCommand(CrocodileSubsystem subsystem,DoubleSupplier motorSpeed) {
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
