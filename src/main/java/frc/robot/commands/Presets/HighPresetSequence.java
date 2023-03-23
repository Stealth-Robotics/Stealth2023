package frc.robot.commands.Presets;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.RotatorToPosition;
import frc.robot.commands.TelescopeToPosition;
import frc.robot.subsystems.CrocodileSubsystem;
import frc.robot.subsystems.RotatorSubsystem;
import frc.robot.subsystems.TelescopeSubsystem;
import frc.robot.subsystems.CrocodileSubsystem.GamePiece;
import frc.robot.subsystems.CrocodileSubsystem.WristPosition;
import frc.robot.subsystems.RotatorSubsystem.RotatorPosition;
import frc.robot.subsystems.TelescopeSubsystem.TelescopePosition;

public class HighPresetSequence extends SequentialCommandGroup {
    private DoubleSupplier intake;
    private double multiplier = 1;

    public HighPresetSequence(TelescopeSubsystem telescope, RotatorSubsystem rotator, CrocodileSubsystem crocodile,
            DoubleSupplier intake, Supplier<GamePiece> gamePiece) {
        this.intake = intake;
        addRequirements(rotator, telescope, crocodile);

        if (gamePiece.get() == GamePiece.CUBE) {
            multiplier = -1;
        }
        addCommands(
                new InstantCommand(() -> crocodile.setIntakeSpeed(0.25 * multiplier)),
                new RotatorToPosition(rotator, telescope, RotatorPosition.HIGH_SCORE).withTimeout(2),
                new TelescopeToPosition(telescope, TelescopePosition.HIGH_SCORE).withTimeout(2));
        switch (gamePiece.get()) {
            case CONE:
                addCommands(
                        crocodile.setWristToPositionCommand(WristPosition.CONE_SCORE).withTimeout(2));
                break;
            case CUBE:
                addCommands(
                        crocodile.setWristToPositionCommand(WristPosition.CUBE_SCORE).withTimeout(2));
                break;
        }
    }
}
