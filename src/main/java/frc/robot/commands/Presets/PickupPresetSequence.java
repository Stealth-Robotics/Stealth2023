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

public class PickupPresetSequence extends SequentialCommandGroup {
    public PickupPresetSequence(TelescopeSubsystem telescope, RotatorSubsystem rotator, CrocodileSubsystem crocodile, BooleanSupplier button, BooleanSupplier gamePieice) {
        addRequirements(telescope,rotator,crocodile);
        addCommands(
            new TelescopeToPosition(telescope, TelescopePosition.GROUND_PICKUP).withTimeout(2),
            new RotatorToPosition(rotator, telescope, RotatorPosition.GROUND_PICKUP).withTimeout(2)
        );
        if (gamePieice.getAsBoolean()){
            addCommands(crocodile.setWristToPositionCommand(WristPosition.CONE_PICKUP).withTimeout(2));
        }
        else {
            addCommands(crocodile.setWristToPositionCommand(WristPosition.CUBE_PICKUP).withTimeout(2));
        }
        addCommands(new AutoIntakeCommand(crocodile, 1, button));
    }
    public PickupPresetSequence(TelescopeSubsystem telescope, RotatorSubsystem rotator, CrocodileSubsystem crocodile, BooleanSupplier button) {
        addRequirements(telescope,rotator,crocodile);
        addCommands(
            new TelescopeToPosition(telescope, TelescopePosition.GROUND_PICKUP).withTimeout(2),
            new RotatorToPosition(rotator, telescope, RotatorPosition.GROUND_PICKUP).withTimeout(2)
        );
        addCommands(crocodile.setWristToPositionCommand(WristPosition.CONE_PICKUP).withTimeout(2));
        addCommands(new AutoIntakeCommand(crocodile, 1, button));
    }
}
