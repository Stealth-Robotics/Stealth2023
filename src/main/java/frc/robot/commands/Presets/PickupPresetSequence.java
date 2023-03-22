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

public class PickupPresetSequence extends SequentialCommandGroup {
    public PickupPresetSequence(TelescopeSubsystem telescope, RotatorSubsystem rotator, CrocodileSubsystem crocodile, BooleanSupplier button) {
        addRequirements(telescope,rotator,crocodile);
        addCommands(
            new TelescopeToPosition(telescope, telescope.ticksToPercent(10000)).withTimeout(2),
            new RotatorToPosition(rotator, telescope, 72).withTimeout(2),
            crocodile.setWristToPositionCommand(WristPosition.CONE_PICKUP).withTimeout(2),
            new AutoIntakeCommand(crocodile, 1, button)
        );
    }
}
