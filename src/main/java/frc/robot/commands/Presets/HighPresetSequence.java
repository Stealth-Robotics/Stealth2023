package frc.robot.commands.Presets;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.RotatorToPosition;
import frc.robot.commands.TelescopeToPosition;
import frc.robot.subsystems.CrocodileSubsystem;
import frc.robot.subsystems.RotatorSubsystem;
import frc.robot.subsystems.TelescopeSubsystem;
import frc.robot.subsystems.CrocodileSubsystem.WristPosition;

public class HighPresetSequence extends SequentialCommandGroup {
    public HighPresetSequence(TelescopeSubsystem telescope, RotatorSubsystem rotator, CrocodileSubsystem crocodile) {
        addRequirements(rotator,telescope,crocodile);
        addCommands(
            new RotatorToPosition(rotator, telescope, 221).withTimeout(2),
            new TelescopeToPosition(telescope, telescope.ticksToPercent(37349)).withTimeout(2),
            crocodile.setWristToPositionCommand(WristPosition.CONE_SCORE).withTimeout(2)
            // new InstantCommand(()->crocodile.openChomper())
        );
    }
}
