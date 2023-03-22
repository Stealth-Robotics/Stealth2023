package frc.robot.commands.Presets;

import java.util.function.BooleanSupplier;

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
    public SubstationPickupPresetSequence(TelescopeSubsystem telescope, RotatorSubsystem rotator, CrocodileSubsystem crocodile, BooleanSupplier button) {
        addRequirements(telescope,rotator,crocodile);
        addCommands(
            new RotatorToPosition(rotator, telescope, RotatorPosition.SHELF_PICKUP).withTimeout(2),
            new TelescopeToPosition(telescope, TelescopePosition.SHELF_PICKUP).withTimeout(2),
            crocodile.setWristToPositionCommand(WristPosition.CONE_SHELF).withTimeout(2),
            new AutoIntakeCommand(crocodile, 1, button)
        );
    }
}
