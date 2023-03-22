package frc.robot.commands.Presets;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AutoIntakeCommand;
import frc.robot.subsystems.CrocodileSubsystem;
import frc.robot.subsystems.RotatorSubsystem;
import frc.robot.subsystems.TelescopeSubsystem;

public class GroundIntakePreset extends SequentialCommandGroup{
    public GroundIntakePreset(TelescopeSubsystem telescope, RotatorSubsystem rotator, CrocodileSubsystem crocodile) {
        addCommands(
            //rotate down to ground pickup level
            //extend telescope
            //TODO: tune speed
            new AutoIntakeCommand(crocodile, 0.5)
        );
    }
    
}
