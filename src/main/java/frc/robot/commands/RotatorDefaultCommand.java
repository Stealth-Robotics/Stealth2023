package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants;
import frc.robot.subsystems.RotatorSubsystem;
import frc.robot.subsystems.TelescopeSubsystem;

public class RotatorDefaultCommand extends CommandBase {
    private final RotatorSubsystem rotator;
    private final DoubleSupplier joystick;
    private final TelescopeSubsystem telescope;
    private final Debouncer crying = new Debouncer(0.5, Debouncer.DebounceType.kBoth);
    private final Command resetTelescopeCmd;

    public RotatorDefaultCommand(RotatorSubsystem rotator, TelescopeSubsystem telescope, DoubleSupplier joystick) {
        this.rotator = rotator;
        this.joystick = joystick;
        this.telescope = telescope;
        addRequirements(rotator);

        resetTelescopeCmd = new ResetTelescope(telescope);
    }

    @Override
    public void initialize() {
        rotator.setToCurrentPosition();
    }

    @Override
    public void execute() {
        double joystickVal = joystick.getAsDouble();
        // If we are within the deadband, do nothing
        if (Math.abs(joystickVal) < Constants.RotatorConstants.RotatorJoystickDeadband) {
            return;
        }
        // If the telescope is extended, and the joystick is being pushed a lot for a
        // perioid of time, reset the telescope
        if (telescope.getCurrentPosition() > 2048 && crying.calculate(Math.abs(joystickVal) > 0.7)) {
            if (!resetTelescopeCmd.isScheduled()) {
                CommandScheduler.getInstance().schedule(resetTelescopeCmd);
                crying.calculate(false); // Make them push thru it again!
            }
        }
        // If the telescope is extended, and the joystick is being pushed a little,
        // allow them to move it (but only a little)
        else if (Math.abs(joystickVal) <= 0.4 && telescope.getCurrentPosition() > 2048) {
            rotator.setGoal(MathUtil.clamp(
                    (rotator.getSetpoint() + joystickVal * Constants.RotatorConstants.ROTATOR_SPEED_MULTIPLIER),
                    Constants.RotatorConstants.LOW_BOUND, Constants.RotatorConstants.HIGH_BOUND));
        }
        // Otherwise, just move the rotator normally
        else {
            rotator.setGoal(MathUtil.clamp(
                    (rotator.getSetpoint() + joystickVal * Constants.RotatorConstants.ROTATOR_SPEED_MULTIPLIER),
                    Constants.RotatorConstants.LOW_BOUND, Constants.RotatorConstants.HIGH_BOUND));
        }

    }

}
