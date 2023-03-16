package frc.robot.commands.DefaultCommands;

import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.CrocodileSubsystem;

public class CrocodileDefaultCommand extends CommandBase {
    private final CrocodileSubsystem subsystem;
    private final DoubleSupplier trigger;
    private boolean beamBreakTracker = false;
    private final Debouncer debouncer;
    private final Timer timer;
    DoubleConsumer giveHapticFeedback;

    public CrocodileDefaultCommand(CrocodileSubsystem subsystem, DoubleSupplier trigger,
            DoubleConsumer giveHapticFeedback) {
        this.subsystem = subsystem;
        this.trigger = trigger;
        timer = new Timer();
        
        debouncer = new Debouncer(0.5, DebounceType.kBoth);
        this.giveHapticFeedback = giveHapticFeedback;
        addRequirements(subsystem);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

        subsystem.setIntakeSpeed(trigger.getAsDouble());
        // sets rumble if beam break is broken for 0.5 seconds and is not already
        // rumbling
        if (debouncer.calculate(subsystem.getBeamBreak()) && !beamBreakTracker) {
            // get current time and do something to rumble
            timer.start();
            beamBreakTracker = true;

            giveHapticFeedback.accept(0.5);
        }
        if (timer.hasElapsed(0.5)) {
            // stop rumble after 500ms
            timer.stop();
            timer.reset();
            giveHapticFeedback.accept(0);
        }
        if (!subsystem.getBeamBreak()) {
            // reset beam break tracker if beam break is not broken for 0.5 seconds
            beamBreakTracker = false;
        }
    }
}
