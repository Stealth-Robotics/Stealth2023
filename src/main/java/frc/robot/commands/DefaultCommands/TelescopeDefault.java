package frc.robot.commands.DefaultCommands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TelescopeSubsystem;
import frc.robot.subsystems.TelescopeSubsystem.TelescopeBoundState;

public class TelescopeDefault extends CommandBase {

    private TelescopeSubsystem telescopeSubsystem;
    private DoubleSupplier joystickSupplier;

    public TelescopeDefault(TelescopeSubsystem telescopeSubsystem, DoubleSupplier joystickSupplier) {
        this.telescopeSubsystem = telescopeSubsystem;
        this.joystickSupplier = joystickSupplier;
        addRequirements(telescopeSubsystem);

    }

    @Override
    public void initialize() {
        // telescopeSubsystem.setToCurrentPosition();
    }

    @Override
    public void execute() {
        double joystickInput = joystickSupplier.getAsDouble();

        if (Math.abs(joystickInput) > 0.05) {
            if (telescopeSubsystem.inBounds() == TelescopeBoundState.IN_BOUNDS) {
                telescopeSubsystem.setSpeed(MathUtil.clamp(joystickInput, -0.3, 0.3));
            } else if (telescopeSubsystem.inBounds() == TelescopeBoundState.OVER_UPPER_BOUND) {
                telescopeSubsystem.setSpeed(MathUtil.clamp(joystickInput, -0.3, 0));
            } else if (telescopeSubsystem.inBounds() == TelescopeBoundState.UNDER_LOWER_BOUND) {
                telescopeSubsystem.setSpeed(MathUtil.clamp(joystickInput, 0, 0.3));
            }
            telescopeSubsystem.setRunPID(false);
        } else {
            telescopeSubsystem.setRunPID(true);

            telescopeSubsystem.setToCurrentPosition();
        }
    }
}
