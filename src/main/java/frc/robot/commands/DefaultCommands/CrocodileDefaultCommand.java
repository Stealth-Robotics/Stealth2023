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
        //WRIST MANUAL CONTROL
        if (Math.abs(wristTrigger.getAsDouble()) > 0.05) {
            subsystem.setWristSpeed(MathUtil.clamp(wristTrigger.getAsDouble(), -0.5, 0.5));
            subsystem.setRunPID(false);
            subsystem.setToCurrentPosition();
        }
        else {
            //subsystem.setRunPID(true);
            subsystem.setWristSpeed(0);
        }  
        //INTAKE
        double power = trigger.getAsDouble();
        if (!subsystem.getBeamBreak()){
            power += 0.25;
        }
        if(subsystem.getGamePiece() == CrocodileSubsystem.GamePiece.CONE){
            subsystem.setIntakeSpeed(power);
        }
        else{
            subsystem.setIntakeSpeed(-power);
        }
        


        //RUMBLE STUFF BLEOW
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
