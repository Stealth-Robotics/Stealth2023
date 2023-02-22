package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.Rotator;

public class ArmDefaultCommand extends CommandBase {
    private final Rotator rotator;
    private final DoubleSupplier joystick;

    public ArmDefaultCommand(Rotator rotator, DoubleSupplier joystick) {
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
        double lowBound = ArmConstants.LOW_BOUND_INTAKE;
        rotator.setGoal(MathUtil.clamp((rotator.getSetpoint() + joystick.getAsDouble() * ArmConstants.ROTATOR_SPEED),
                lowBound, ArmConstants.HIGH_BOUND));
    }

}
