package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeDefaultCommand extends CommandBase{
    private IntakeSubsystem intakeSubsystem;
    private final DoubleSupplier rightTrigger;
    private boolean beamBreakTracker = false;
    private final XboxController driverController;
    private final Debouncer debouncer;
    public IntakeDefaultCommand(IntakeSubsystem intakeSubsystem, DoubleSupplier rightTrigger){
        this.intakeSubsystem = intakeSubsystem;
        this.rightTrigger = rightTrigger;
        //TODO: get driver controller port
        driverController = new XboxController(0);
        debouncer = new Debouncer(0.5, DebounceType.kBoth);
        addRequirements(intakeSubsystem);
    }
    @Override
    public void execute() {
        //set intake speed to right trigger
        long startTime = 0;
        intakeSubsystem.setIntakeSpeed(rightTrigger.getAsDouble());
        //sets rumble if beam break is broken for 0.5 seconds and is not already rumbling
        if(debouncer.calculate(intakeSubsystem.getBeamBreak()) && !beamBreakTracker){
            //get current time and set rumble
            startTime = System.currentTimeMillis();
            driverController.setRumble(RumbleType.kBothRumble, 0.5);
            beamBreakTracker = true;
        }
        if(System.currentTimeMillis() - startTime > 500){
            //stop rumble after 500ms
            driverController.setRumble(RumbleType.kBothRumble, 0);
        }
        if(!debouncer.calculate(intakeSubsystem.getBeamBreak())){
            //reset beam break tracker if beam break is not broken for 0.5 seconds
            beamBreakTracker = false;
        }
    }
    
    
}
