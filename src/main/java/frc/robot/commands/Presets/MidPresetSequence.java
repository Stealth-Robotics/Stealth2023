package frc.robot.commands.Presets;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.RotatorToPosition;
import frc.robot.commands.TelescopeToPosition;
import frc.robot.subsystems.CrocodileSubsystem;
import frc.robot.subsystems.RotatorSubsystem;
import frc.robot.subsystems.TelescopeSubsystem;
import frc.robot.subsystems.CrocodileSubsystem.WristPosition;
import frc.robot.subsystems.RotatorSubsystem.RotatorPosition;

public class MidPresetSequence extends SequentialCommandGroup {
    public MidPresetSequence(TelescopeSubsystem telescope, RotatorSubsystem rotator, CrocodileSubsystem crocodile) {
        addRequirements(rotator,telescope,crocodile);
        addCommands(
            new RotatorToPosition(rotator, telescope, RotatorPosition.HIGH_SCORE).withTimeout(2),
            crocodile.setWristToPositionCommand(WristPosition.CONE_SCORE).withTimeout(2)
            // new InstantCommand(()->crocodile.openChomper())
        );
    }
}
