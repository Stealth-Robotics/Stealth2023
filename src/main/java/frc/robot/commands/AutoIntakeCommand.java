package frc.robot.commands;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;

public class AutoIntakeCommand extends CommandBase{
    private final IntakeSubsystem intakeSubsystem;
    private final double speed;
    public AutoIntakeCommand(IntakeSubsystem intakeSubsystem, double speed){
        this.intakeSubsystem = intakeSubsystem;
        this.speed = speed;
        addRequirements(intakeSubsystem);
    }
    @Override
    public void initialize() {
        intakeSubsystem.setIntakeSpeed(speed);
    }
    @Override
    public boolean isFinished() {
        Debouncer debouncer = new Debouncer(0.5, DebounceType.kBoth);
        //will return true if beam break is true for 0.5 seconds
        return debouncer.calculate(intakeSubsystem.getBeamBreak());
    }
    @Override
    public void end(boolean interrupted) {
        intakeSubsystem.setIntakeSpeed(0);
    }

    
}
