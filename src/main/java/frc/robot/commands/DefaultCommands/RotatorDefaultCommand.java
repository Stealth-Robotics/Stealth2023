package frc.robot.commands.DefaultCommands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.SharedConstants;
import frc.robot.commands.ResetTelescope;
import frc.robot.subsystems.RotatorSubsystem;
import frc.robot.subsystems.TelescopeSubsystem;

public class RotatorDefaultCommand extends CommandBase {
    private final double ROTATOR_JOYSTICK_DEADBAND = 0.5;

    private final TelescopeSubsystem telescope;
    private final RotatorSubsystem rotator;
    // One of the joystick axes of the controller
    private final DoubleSupplier joystick;
    // Debouncer to measure the depression of the joystick
    private final Debouncer crying = new Debouncer(0.5, Debouncer.DebounceType.kBoth);
    // Command to reset the telescope
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
        // Start with the rotator at the current position
        //rotator.setToCurrentPosition();
    }

    @Override
    public void execute() {

        double joystickVal = joystick.getAsDouble();
        // // If we are within the deadband, do nothing
        // if (Math.abs(joystickVal) < ROTATOR_JOYSTICK_DEADBAND) {
        //     return;
        // }
        // // If the telescope is extended, and the joystick is being pushed a lot for a
        // // // perioid of time, reset the telescope
        // if (telescope.getExtensionPercent() > 0.1 && crying.calculate(Math.abs(joystickVal) > 0.7)) {
        //     if (!resetTelescopeCmd.isScheduled()) {
        //         CommandScheduler.getInstance().schedule(resetTelescopeCmd);
        //         crying.calculate(false); // Make them push thru it again!
        //     }
        // }
        // // // If the telescope is extended, and the joystick is being pushed a little,
        // // // allow them to move it (but only a little)
        // else if (telescope.getExtensionPercent() > 0.1) {
        //     rotator.setSpeed(MathUtil.clamp(joystickVal * 0.5, -0.3, 0.3));
        //     rotator.setToCurrentPosition();
        // }
        // // Otherwise, just move the rotator normally
        // else {
            // if (joystickVal < 0 && rotator.getMeasurementDegrees() < 0)
            //     return;
            // if (joystickVal > 0 && rotator.getMeasurementDegrees() > 250)
            //     return;
         if (Math.abs(joystickVal) > 0.05){
            rotator.setSpeed(MathUtil.clamp(joystickVal * 0.5, -0.5, 0.5));
            rotator.setRunPID(false);
         }
         else
         {
            rotator.setRunPID(true);
         }
         rotator.setToCurrentPosition();

            
        // }

    }

}
