package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.CrocodileSubsystem;

public class CrocodileDefaultCommand extends CommandBase {
    private final CrocodileSubsystem subsystem;
    private final DoubleSupplier trigger;
    private final XboxController driverController;
    private boolean beamBreakTracker = false;
    private final Debouncer debouncer;

    public CrocodileDefaultCommand(CrocodileSubsystem subsystem, DoubleSupplier trigger) {
        this.subsystem = subsystem;
        this.trigger = trigger;
        //TODO: get driver controller port
        //controller to use for rumble
        driverController = new XboxController(0);
        debouncer = new Debouncer(0.5);
        addRequirements(subsystem);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        long startTime = 0;
        subsystem.setIntakeSpeed(trigger.getAsDouble());
        //sets rumble if beam break is broken for 0.5 seconds and is not already rumbling
        if(debouncer.calculate(subsystem.getBeamBreak()) && !beamBreakTracker){
            //get current time and set rumble
            startTime = System.currentTimeMillis();
            driverController.setRumble(XboxController.RumbleType.kLeftRumble, 0.5);
            driverController.setRumble(XboxController.RumbleType.kRightRumble, 0.5);
            beamBreakTracker = true;
        }
        if(System.currentTimeMillis() - startTime > 500){
            //stop rumble after 500ms
            driverController.setRumble(XboxController.RumbleType.kLeftRumble, 0);
            driverController.setRumble(XboxController.RumbleType.kRightRumble, 0);
        }
        if(debouncer.calculate(!subsystem.getBeamBreak())){
            //reset beam break tracker if beam break is not broken for 0.5 seconds
            beamBreakTracker = false;
        }
    }
}
