package frc.robot.commands.Presets;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AutoIntakeCommand;
import frc.robot.commands.RotatorToPosition;
import frc.robot.commands.TelescopeToPosition;
import frc.robot.subsystems.CrocodileSubsystem;
import frc.robot.subsystems.RotatorSubsystem;
import frc.robot.subsystems.TelescopeSubsystem;
import frc.robot.subsystems.CrocodileSubsystem.WristPosition;
import frc.robot.subsystems.TelescopeSubsystem.TelescopePosition;

public class StowPresetSequence extends SequentialCommandGroup {
    public StowPresetSequence(TelescopeSubsystem telescope, RotatorSubsystem rotator, CrocodileSubsystem crocodile, BooleanSupplier gamePiece) {
        addRequirements(telescope,rotator,crocodile);
        double multiplier = gamePiece.getAsBoolean() == true ? 1 : -1;
        addCommands(
            new TelescopeToPosition(telescope, TelescopePosition.RETRACTED).withTimeout(2),
            new RotatorToPosition(rotator, telescope, 90).withTimeout(2),
            crocodile.setWristToPositionCommand(WristPosition.CONE_SCORE).withTimeout(2)
        );
        deadlineWith(new RunCommand(() -> crocodile.setIntakeSpeed(0.25 * multiplier)));
    }
    public StowPresetSequence(TelescopeSubsystem telescope, RotatorSubsystem rotator, CrocodileSubsystem crocodile) {
        addRequirements(telescope,rotator,crocodile);
        addCommands(
            new TelescopeToPosition(telescope, TelescopePosition.RETRACTED).withTimeout(2),
            new RotatorToPosition(rotator, telescope, 90).withTimeout(2),
            crocodile.setWristToPositionCommand(WristPosition.CONE_SCORE).withTimeout(2)
        );
    }
}
