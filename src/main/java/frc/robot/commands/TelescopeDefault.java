package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.TelescopeSubsystem;

public class TelescopeDefault extends CommandBase {

    private TelescopeSubsystem telescopeSubsystem;
    private DoubleSupplier joystickSupplier;

    public TelescopeDefault(TelescopeSubsystem telescopeSubsystem, DoubleSupplier joystickSupplier) {
        this.telescopeSubsystem = telescopeSubsystem;
        this.joystickSupplier = joystickSupplier;
        addRequirements(telescopeSubsystem);
    }

    @Override
    public void execute() {
        double joystickInput = joystickSupplier.getAsDouble();

        if (Math.abs(joystickInput) > 0.05) {
            telescopeSubsystem.setSetpoint(telescopeSubsystem.getSetpoint()
                    + (joystickInput * Constants.TelescopeConstants.TELESCOPE_SPEED_MULTIPLIER));
        }

        else {
            telescopeSubsystem.setPosition(telescopeSubsystem.getSetpoint());
        }
    }
}
