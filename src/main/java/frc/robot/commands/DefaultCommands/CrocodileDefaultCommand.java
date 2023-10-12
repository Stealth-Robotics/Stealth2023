package frc.robot.commands.DefaultCommands;

import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.CrocodileSubsystem;
import frc.robot.subsystems.Gamepiece;

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
        if (Math.abs(wristTrigger.getAsDouble()) > 0.05) {
            subsystem.setWristSpeed(MathUtil.clamp(wristTrigger.getAsDouble(), -0.5, 0.5));
            subsystem.setRunPID(false);
            subsystem.setToCurrentPosition();
        }
        else {
            subsystem.setRunPID(true);
        }          
    }
}
