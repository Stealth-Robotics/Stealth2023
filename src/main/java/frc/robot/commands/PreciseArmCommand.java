package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Rotation;

public class PreciseArmCommand extends CommandBase {
    private DoubleSupplier joystick;
    private Rotation arm;
    private long startTime, nextTime;

    public PreciseArmCommand(Rotation arm, DoubleSupplier joystick) {
        this.arm = arm;
        this.joystick = joystick;
    }

    @Override
    public void initialize() {
        startTime = System.nanoTime() / (long) Math.pow(10, 6);
    }

    @Override
    public void execute() {
        nextTime = System.nanoTime() / (long) Math.pow(10, 6);
        long deltaTime = nextTime - startTime;
        startTime = System.nanoTime();
        arm.setSetpoint(joystick.getAsDouble() * deltaTime * Constants.ArmConstants.ARM_TICKS_PER_MILLISECOND);
        arm.updatePosition();
    }
}
