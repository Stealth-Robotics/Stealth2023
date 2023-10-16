package frc.robot.commands;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import java.util.function.BooleanSupplier;
import frc.robot.subsystems.Gamepiece;
import frc.robot.subsystems.IntakeSubsystem;

public class AutoIntakeCommand extends CommandBase {
    private final IntakeSubsystem intakeSubsystem;
    private final double speed;
    private final Timer timer;
    private final Debouncer debouncer;
    private final BooleanSupplier stopIntake;
    private final Gamepiece gamePiece;

    public AutoIntakeCommand(IntakeSubsystem intakeSubsystem, double speed,
            BooleanSupplier stopIntake, Gamepiece gamePiece) {
        this.intakeSubsystem = intakeSubsystem;
        this.speed = speed;
        debouncer = new Debouncer(0.5, DebounceType.kFalling);
        timer = new Timer();
        this.stopIntake = stopIntake;
        intakeSubsystem.setGamePiece(gamePiece);
        this.gamePiece = intakeSubsystem.getGamePiece();
        addRequirements(intakeSubsystem);
    }

    // overload autointke and set stopIntake to null, use for auto
    public AutoIntakeCommand(IntakeSubsystem intakeSubsystem, double speed,
            Gamepiece gamePiece) {
        this(intakeSubsystem, speed, null, gamePiece);
    }

    // overload autointake and set gamepiece to getGamePiece
    public AutoIntakeCommand(IntakeSubsystem intakeSubsystem,  double speed,
            BooleanSupplier stopIntake) {
        this(intakeSubsystem, speed, stopIntake, intakeSubsystem.getGamePiece());
    }

    @Override
    public void initialize() {
        // TODO: check if this is the right negation
        if (intakeSubsystem.getGamePiece() == Gamepiece.CONE) {
            intakeSubsystem.setIntakeSpeed(speed);
        } else {
            intakeSubsystem.setIntakeSpeed(-speed);
        }
    }
    @Override
    public void execute(){

    }
    @Override
    public boolean isFinished() {
        // if outtaking, keep running motors until beam break hasn't been broken for 0.5
        // seconds
        return !intakeSubsystem.getBeamBreak() || stopIntake.getAsBoolean();

    }

    @Override
    public void end(boolean interrupted) {
        intakeSubsystem.setIntakeSpeed(0);

    }

}
