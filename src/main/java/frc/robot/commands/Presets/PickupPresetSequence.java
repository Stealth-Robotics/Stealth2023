package frc.robot.commands.Presets;

import java.util.function.BooleanSupplier;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.RotatorToPosition;
import frc.robot.commands.TelescopeToPosition;
import frc.robot.subsystems.CrocodileSubsystem;
import frc.robot.subsystems.RotatorSubsystem;
import frc.robot.subsystems.TelescopeSubsystem;
import frc.robot.subsystems.Gamepiece;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.CrocodileSubsystem.WristPosition;
import frc.robot.subsystems.RotatorSubsystem.RotatorPosition;
import frc.robot.subsystems.TelescopeSubsystem.TelescopePosition;

public class PickupPresetSequence extends SequentialCommandGroup {
    public PickupPresetSequence(TelescopeSubsystem telescope, RotatorSubsystem rotator, CrocodileSubsystem crocodile,
            IntakeSubsystem intakeSubsystem, BooleanSupplier button) {
        addRequirements(telescope, rotator, crocodile);
        addCommands(
                new TelescopeToPosition(telescope, TelescopePosition.GROUND_PICKUP).withTimeout(2),
                new RotatorToPosition(rotator, telescope, RotatorPosition.GROUND_PICKUP).withTimeout(2),
                new ConditionalCommand(
                        crocodile.setWristToPositionCommand(WristPosition.CONE_PICKUP),
                        crocodile.setWristToPositionCommand(WristPosition.CUBE_PICKUP),
                        () -> intakeSubsystem.getGamePiece() == Gamepiece.CONE).withTimeout(2.5));
    }
}
