package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.CrocodileSubsystem;
import frc.robot.subsystems.RotatorSubsystem;
import frc.robot.subsystems.TelescopeSubsystem;

public class HighPresetSequence extends SequentialCommandGroup {
    public HighPresetSequence(TelescopeSubsystem telescope, RotatorSubsystem rotator, CrocodileSubsystem crocodile) {
        addCommands(
            new RotatorToPosition(rotator, telescope, 230),
            new TelescopeToPosition(telescope, 80000)
            // new InstantCommand(()->crocodile.openChomper())
        );
    }
}
