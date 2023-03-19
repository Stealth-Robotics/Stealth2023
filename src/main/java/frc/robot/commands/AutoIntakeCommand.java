package frc.robot.commands;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.CrocodileSubsystem;
import java.util.function.BooleanSupplier;
import frc.robot.subsystems.CrocodileSubsystem.GamePiece;

public class AutoIntakeCommand extends CommandBase {
    private final CrocodileSubsystem crocodileSubsystem;
    private final double speed;
    private final Timer timer;
    private final Debouncer debouncer;
    private final BooleanSupplier stopIntake;
    private final GamePiece gamePiece;

    public AutoIntakeCommand(CrocodileSubsystem crocodileSubsystem, double speed, BooleanSupplier stopIntake) {
        this.crocodileSubsystem = crocodileSubsystem;
        this.speed = speed;
        debouncer = new Debouncer(0.5, DebounceType.kFalling);
        timer = new Timer();
        this.stopIntake = stopIntake;
        this.gamePiece = crocodileSubsystem.getGamePiece();
        addRequirements(crocodileSubsystem);
    }

    // overload autointke and set stopIntake to null
    public AutoIntakeCommand(CrocodileSubsystem crocodileSubsystem, double speed) {
        this(crocodileSubsystem, speed, null);
    }

    @Override
    public void initialize() {
        //TODO: check if this is the right negation
        if(gamePiece == GamePiece.CONE){
            crocodileSubsystem.setIntakeSpeed(speed);
        }
        else{
            crocodileSubsystem.setIntakeSpeed(-speed);
        }
    }

    @Override
    public boolean isFinished() {
        // if outtaking, keep running motors until beam break hasn't been broken for 0.5
        // seconds
        
        if ((speed < 0 && gamePiece == GamePiece.CONE) || (speed > 0 && gamePiece == GamePiece.CUBE)) {
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
            return stopIntake.getAsBoolean();
        }
        return false;

    }

    @Override
    public void end(boolean interrupted) {
        crocodileSubsystem.setIntakeSpeed(0);
    }

}
