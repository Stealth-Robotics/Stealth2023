package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Rotator;

public class RotatorDefaultCommand extends CommandBase {
    private final Rotator rotator;
    private final DoubleSupplier joystick;

    public RotatorDefaultCommand(Rotator rotator, DoubleSupplier joystick) {
        this.rotator = rotator;
        this.joystick = joystick;
        addRequirements(rotator);
    }
    @Override
    public void initialize() {
        rotator.setGoal(Constants.RotatorConstants.ENCODER_OFFSET);
    }
    @Override
    public void execute() {
        double lowBound = Constants.RotatorConstants.LOW_BOUND_INTAKE;
        rotator.setGoal(MathUtil.clamp((rotator.getSetpoint() + joystick.getAsDouble() * Constants.RotatorConstants.ROTATOR_SPEED_MULTIPLIER),
                lowBound, Constants.RotatorConstants.HIGH_BOUND));
    }

}
