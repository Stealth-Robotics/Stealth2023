package frc.robot.commands.DefaultCommands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.CrocodileSubsystem;
import frc.robot.subsystems.CrocodileSubsystem.WristBoundState;

public class CrocodileDefaultCommand extends CommandBase {
    private final CrocodileSubsystem subsystem;
    private final DoubleSupplier wristTrigger;
    private final BooleanSupplier override;

    public CrocodileDefaultCommand(CrocodileSubsystem subsystem, DoubleSupplier manualWrist, BooleanSupplier override) {
        this.subsystem = subsystem;
        this.wristTrigger = manualWrist;
        this.override = override;
        addRequirements(subsystem);
    }

    

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        //WRIST MANUAL CONTROL
        double triggerInput = wristTrigger.getAsDouble();
        // if(override.getAsBoolean() && Math.abs(triggerInput) > 0.1){
        //     subsystem.setWristSpeed(MathUtil.clamp(wristTrigger.getAsDouble(), -0.5, 0.5));
        //     subsystem.setRunPID(false);
        // }
        // //TODO: CHECK IF THIS WORKS
        // else if (Math.abs(triggerInput) > 0.1) {
        //     if(subsystem.inBounds() == WristBoundState.IN_BOUNDS) {
        //         subsystem.setWristSpeed(MathUtil.clamp(wristTrigger.getAsDouble(), -0.5, 0.5));
        //     } else if (subsystem.inBounds() == WristBoundState.OVER_UPPER_BOUND) {
        //         subsystem.setWristSpeed(MathUtil.clamp(wristTrigger.getAsDouble(), 0, 0.5)); 
        //     } else if (subsystem.inBounds() == WristBoundState.UNDER_LOWER_BOUND) {
        //         subsystem.setWristSpeed(MathUtil.clamp(wristTrigger.getAsDouble(), -0.5, 0));
        //     }
        //     subsystem.setRunPID(false);
        // }

        if(Math.abs(triggerInput) > 0.1){
            subsystem.setWristSpeed(MathUtil.clamp(triggerInput, -0.5, 0.5));
            subsystem.setRunPID(false);
            subsystem.setToCurrentPosition();
        }
        else {
            //set to current position, won't switch to PID till loop aftere trigger is released
            subsystem.setRunPID(true);
        }          
    }
}
