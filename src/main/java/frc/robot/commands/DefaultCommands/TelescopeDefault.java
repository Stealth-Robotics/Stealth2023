package frc.robot.commands.DefaultCommands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TelescopeSubsystem;
import frc.robot.subsystems.TelescopeSubsystem.TelescopeBoundState;

public class TelescopeDefault extends CommandBase {

    boolean setpointSet = true;

    private TelescopeSubsystem telescopeSubsystem;
    private DoubleSupplier joystickSupplier;
    private BooleanSupplier override;
    public TelescopeDefault(TelescopeSubsystem telescopeSubsystem, DoubleSupplier joystickSupplier, BooleanSupplier override) {
    this.telescopeSubsystem = telescopeSubsystem;
        this.joystickSupplier = joystickSupplier;
        this.override = override;
        addRequirements(telescopeSubsystem);

    }

    @Override
    public void initialize() {
        // telescopeSubsystem.setToCurrentPosition();
    }

    @Override
    public void execute() {
        double joystickInput = joystickSupplier.getAsDouble();
        if (override.getAsBoolean()){
            telescopeSubsystem.setSpeed(MathUtil.clamp(joystickInput, -0.3, 0.3));
            telescopeSubsystem.setRunPID(false);
            setpointSet = false;
        }
        else if (Math.abs(joystickInput) > 0.1) {
            if (telescopeSubsystem.inBounds() == TelescopeBoundState.IN_BOUNDS) {
                telescopeSubsystem.setSpeed(MathUtil.clamp(joystickInput, -0.3, 0.3));
            } else if (telescopeSubsystem.inBounds() == TelescopeBoundState.OVER_UPPER_BOUND) {
                telescopeSubsystem.setSpeed(MathUtil.clamp(joystickInput, -0.3, 0));
            } else if (telescopeSubsystem.inBounds() == TelescopeBoundState.UNDER_LOWER_BOUND) {
                telescopeSubsystem.setSpeed(MathUtil.clamp(joystickInput, 0, 0.3));
            }
            telescopeSubsystem.setRunPID(false);
            setpointSet = false;
        } else if (!setpointSet) {
            telescopeSubsystem.setRunPID(true);
            setpointSet = true;

            telescopeSubsystem.setToCurrentPosition();
        }
    }
}
