package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Swerve.DrivebaseSubsystem;

public class DriveSetSpeedCommand extends SequentialCommandGroup{

    public DriveSetSpeedCommand(DrivebaseSubsystem drive, double speed){
        addCommands(new InstantCommand(() -> drive.setMultiplier(speed)));
    }
    
}
