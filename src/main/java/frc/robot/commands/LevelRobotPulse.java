package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Swerve.DrivebaseSubsystem;

public class LevelRobotPulse extends SequentialCommandGroup{

    public LevelRobotPulse(DrivebaseSubsystem drive){
        LevelRobot level = new LevelRobot(drive);
        addCommands(
            new RepeatCommand(
                new SequentialCommandGroup(
                    new WaitCommand(0.6),
                    level.withTimeout(0.5)
                )
            ).until(() -> level.isFinished())
        );
    }
    
}
