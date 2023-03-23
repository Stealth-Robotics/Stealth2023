package frc.robot.commands.Presets;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AutoIntakeCommand;
import frc.robot.commands.RotatorToPosition;
import frc.robot.commands.TelescopeToPosition;
import frc.robot.subsystems.CrocodileSubsystem;
import frc.robot.subsystems.RotatorSubsystem;
import frc.robot.subsystems.TelescopeSubsystem;
import frc.robot.subsystems.CrocodileSubsystem.GamePiece;
import frc.robot.subsystems.CrocodileSubsystem.WristPosition;
import frc.robot.subsystems.RotatorSubsystem.RotatorPosition;
import frc.robot.subsystems.TelescopeSubsystem.TelescopePosition;

public class SubstationPickupPresetSequence extends SequentialCommandGroup {
    public SubstationPickupPresetSequence(TelescopeSubsystem telescope, RotatorSubsystem rotator,
            CrocodileSubsystem crocodile, BooleanSupplier button, Supplier<GamePiece> gamePiece) {
        addRequirements(telescope, rotator, crocodile);
        addCommands(
                new RotatorToPosition(rotator, telescope, RotatorPosition.SHELF_PICKUP).withTimeout(2),
                new TelescopeToPosition(telescope, TelescopePosition.SHELF_PICKUP).withTimeout(2));
        switch (gamePiece.get()) {
            case CONE:
                addCommands(
                        crocodile.setWristToPositionCommand(WristPosition.CONE_SHELF).withTimeout(2));
                break;
            case CUBE:
                addCommands(
                        crocodile.setWristToPositionCommand(WristPosition.CUBE_SHELF).withTimeout(2));
                break;
        }
        addCommands(new AutoIntakeCommand(crocodile, 1, button));
    }
}
