package frc.robot.commands;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.CrocodileSubsystem;

public class AutoIntakeCommand extends CommandBase {
    private final CrocodileSubsystem crocodileSubsystem;
    private final double speed;
    private final Debouncer debouncer;

    public AutoIntakeCommand(CrocodileSubsystem crocodileSubsystem, double speed) {
        this.crocodileSubsystem = crocodileSubsystem;
        this.speed = speed;
        debouncer = new Debouncer(0.5, DebounceType.kBoth);
        addRequirements(crocodileSubsystem);
    }

    @Override
    public void initialize() {
        crocodileSubsystem.setIntakeSpeed(speed);
    }

    @Override
    public boolean isFinished() {
        // will return true if beam break is true for 0.5 seconds
        return debouncer.calculate(crocodileSubsystem.getBeamBreak());
    }

    @Override
    public void end(boolean interrupted) {
        crocodileSubsystem.setIntakeSpeed(0);
    }

}
