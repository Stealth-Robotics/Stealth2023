package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve.DrivebaseSubsystem;

public class LevelRobotPulses extends SequentialCommandGroup{
    private final DrivebaseSubsystem drive;
    
    public LevelRobotPulses(DrivebaseSubsystem drivetrain){
        drive = drivetrain;
        addRequirements(drive);
        addCommands(
            new RepeatCommand(
                new SequentialCommandGroup(
                    new InstantCommand(() -> drivetrain.drive(
                        new Translation2d(1, 0), 0, true, true)),
                    new WaitCommand(0.1))
            )   
        );
    }

   
}