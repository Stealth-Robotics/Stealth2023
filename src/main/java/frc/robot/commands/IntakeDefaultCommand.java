package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

public class IntakeDefaultCommand extends CommandBase {
    private final Intake intake;
    private final DoubleSupplier trigger;

    public IntakeDefaultCommand(Intake intake, DoubleSupplier trigger) {
        this.intake = intake;
        this.trigger = trigger;
        addRequirements(intake);
    }

    @Override
    public void execute() {
        intake.setIntakePower(trigger.getAsDouble());
    }
}
