package frc.robot.commands.DefaultCommands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.CrocodileSubsystem;

public class CrocodileDefaultCommand extends CommandBase {
    private final CrocodileSubsystem subsystem;
    private final DoubleSupplier wristTrigger;
    

    public CrocodileDefaultCommand(CrocodileSubsystem subsystem, DoubleSupplier manualWrist) {
        this.subsystem = subsystem;
        this.wristTrigger = manualWrist;
        addRequirements(subsystem);
    }

    

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        //WRIST MANUAL CONTROL
        if (Math.abs(wristTrigger.getAsDouble()) > 0.1) {
            subsystem.setWristSpeed(MathUtil.clamp(wristTrigger.getAsDouble(), -0.5, 0.5));
            subsystem.setRunPID(false);
            //set to current position, won't switch to PID till loop aftere trigger is released
            subsystem.setToCurrentPosition();
        }
        else {
            subsystem.setRunPID(true);
        }          
    }
}
