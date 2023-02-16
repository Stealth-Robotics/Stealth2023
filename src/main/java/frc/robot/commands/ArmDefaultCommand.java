package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.Rotator;

public class ArmDefaultCommand extends CommandBase {
    private final Rotator rotator;
    private final BooleanSupplier intakeOut;
    private final DoubleSupplier telescopePosition, joystick;

    public ArmDefaultCommand(Rotator rotator, BooleanSupplier intakeOut, DoubleSupplier telescopePosition,
            DoubleSupplier joystick) {
        this.rotator = rotator;
        this.intakeOut = intakeOut;
        this.telescopePosition = telescopePosition;
        this.joystick = joystick;
    }

    @Override
    public void execute() {
        double lowBound = intakeOut.getAsBoolean() ? ArmConstants.LOW_BOUND : ArmConstants.LOW_BOUND_INTAKE;
        rotator.setGoal(MathUtil.clamp((rotator.getSetpoint() + joystick.getAsDouble() * ArmConstants.ROTATOR_SPEED),
                lowBound, ArmConstants.HIGH_BOUND));
    }

}
