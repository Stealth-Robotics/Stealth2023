package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.RotatorSubsystem;

public class RotatorDefaultCommand extends CommandBase {
    private final RotatorSubsystem rotator;
    private final DoubleSupplier joystick;

    public RotatorDefaultCommand(RotatorSubsystem rotator, DoubleSupplier joystick) {
        this.rotator = rotator;
        this.joystick = joystick;
        addRequirements(rotator);

    }

    @Override
    public void initialize() {
        rotator.reset();
    }

    @Override
    public void execute() {
        double joystickVal = joystick.getAsDouble();
        if (Math.abs(joystickVal) > Constants.RotatorConstants.RotatorJoystickDeadband) {
            rotator.setGoal(MathUtil.clamp(
                    (rotator.getSetpoint() + joystickVal * Constants.RotatorConstants.ROTATOR_SPEED_MULTIPLIER),
                    Constants.RotatorConstants.LOW_BOUND, Constants.RotatorConstants.HIGH_BOUND));
        }
    }

}
