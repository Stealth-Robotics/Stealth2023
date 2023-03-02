package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.CrocodileSubsystem;
import frc.robot.subsystems.RotatorSubsystem;
import frc.robot.subsystems.TelescopeSubsystem;

public class LowPresetSequence extends SequentialCommandGroup {
    public LowPresetSequence(TelescopeSubsystem telescope, RotatorSubsystem rotator, CrocodileSubsystem crocodile) {
        addCommands(
            new RotatorToPosition(rotator, telescope, 80),
            new TelescopeToPosition(telescope, 50000),
            new RunCrocodileMotors(crocodile, -0.2),
            new InstantCommand(()->crocodile.openChomper())
        );
    }
}
