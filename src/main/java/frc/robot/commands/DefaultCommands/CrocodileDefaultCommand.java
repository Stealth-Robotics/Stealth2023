package frc.robot.commands.DefaultCommands;

import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.CrocodileSubsystem;
import frc.robot.subsystems.CrocodileSubsystem.GamePiece;

public class CrocodileDefaultCommand extends CommandBase {
    private final CrocodileSubsystem subsystem;
    private final DoubleSupplier trigger;
    private final DoubleSupplier wristTrigger;
    private boolean beamBreakTracker = false;
    private final Debouncer debouncer;
    private final Timer timer;
    DoubleConsumer giveHapticFeedback;

    public CrocodileDefaultCommand(CrocodileSubsystem subsystem, DoubleSupplier trigger, DoubleSupplier manualWrist,
            DoubleConsumer giveHapticFeedback) {
        this.subsystem = subsystem;
        this.trigger = trigger;
        this.wristTrigger = manualWrist;
        timer = new Timer();
        
        debouncer = new Debouncer(0.5, DebounceType.kFalling);
        this.giveHapticFeedback = giveHapticFeedback;
        
        addRequirements(subsystem);
    }

    

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (Math.abs(wristTrigger.getAsDouble()) > 0.05) {
            subsystem.setWristSpeed(MathUtil.clamp(wristTrigger.getAsDouble(), -0.5, 0.5));
            subsystem.setRunPID(false);
            subsystem.setToCurrentPosition();
        }
        else {
            subsystem.setRunPID(true);
        }        
        //TODO: check if this is the right negation
        if(subsystem.getGamePiece() == CrocodileSubsystem.GamePiece.CONE){
            subsystem.setIntakeSpeed(trigger.getAsDouble());
        }
        else{
            subsystem.setIntakeSpeed(-trigger.getAsDouble());
        }
        // sets rumble if beam break is broken for 0.5 seconds and is not already
        // rumbling
        if (!subsystem.getBeamBreak() && !beamBreakTracker) {
            // get current time and do something to rumble
            timer.start();
            beamBreakTracker = true;

            giveHapticFeedback.accept(1.0);
        }
        if (timer.hasElapsed(0.5)) {
            // stop rumble after 500ms
            timer.stop();
            timer.reset();
            giveHapticFeedback.accept(0);
        }
        if (subsystem.getBeamBreak()) {
            // reset beam break tracker if beam break is not broken for 0.5 seconds
            beamBreakTracker = false;
        }
        
    }
}
