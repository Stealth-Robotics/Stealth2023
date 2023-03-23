package frc.robot.commands;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.CrocodileSubsystem;
import java.util.function.BooleanSupplier;

public class AutoIntakeCommand extends CommandBase {
    private final CrocodileSubsystem crocodileSubsystem;
    private final double speed;
    private final Timer timer;
    private final Debouncer debouncer;
    private final BooleanSupplier stopIntake;
    private final BooleanSupplier gamePiece;

    private boolean wasCancelled = false;
    //set stopIntake to null if you don't want to use it
    public AutoIntakeCommand(CrocodileSubsystem crocodileSubsystem, double speed,
            BooleanSupplier stopIntake, BooleanSupplier gamePiece) {
        this.crocodileSubsystem = crocodileSubsystem;
        this.speed = speed;
        debouncer = new Debouncer(0.5, DebounceType.kFalling);
        timer = new Timer();
        this.stopIntake = stopIntake;
        this.gamePiece = gamePiece;
        addRequirements(crocodileSubsystem);
    }
    public AutoIntakeCommand(CrocodileSubsystem crocodileSubsystem, double speed, boolean gamePiece) {
        this(crocodileSubsystem, speed, null, () -> gamePiece);
    }



    @Override
    public void initialize() {
        // TODO: check if this is the right negation
        if (gamePiece.getAsBoolean()) {
            crocodileSubsystem.setIntakeSpeed(speed);
        } else {
            crocodileSubsystem.setIntakeSpeed(-speed);
        }
    }

    @Override
    public boolean isFinished() {
        // if outtaking, keep running motors until beam break hasn't been broken for 0.5
        // seconds

        if ((speed < 0 && gamePiece.getAsBoolean()) || (speed > 0 && !gamePiece.getAsBoolean())) {
            if (crocodileSubsystem.getBeamBreak()) {
                timer.start();
            }
            if (timer.hasElapsed(0.5)) {
                timer.stop();
                timer.reset();
                return true;
            }
        }
        // otherwise, keep running motors until beam break has been broken for 0.5
        // seconds
        else if (!debouncer.calculate(crocodileSubsystem.getBeamBreak())) {
            return true;
        }
        // if none of the above, return the value of stopIntake which is bound to a
        // button, unless it is null
        if (stopIntake != null) {
            wasCancelled = stopIntake.getAsBoolean();
            return stopIntake.getAsBoolean();
        }
        return false;

    }

    @Override
    public void end(boolean interrupted) {
        crocodileSubsystem.setIntakeSpeed(0);

    }

}
