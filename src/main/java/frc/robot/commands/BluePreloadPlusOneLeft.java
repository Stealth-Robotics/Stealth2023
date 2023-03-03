// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.CrocodileSubsystem;
import frc.robot.subsystems.RotatorSubsystem;
import frc.robot.subsystems.TelescopeSubsystem;
import frc.robot.subsystems.Swerve.DrivebaseSubsystem;

public class BluePreloadPlusOneLeft extends SequentialCommandGroup {
  // creates variables for the drivebase and defaultconfig.
  private final DrivebaseSubsystem driveBase;
  private final TrajectoryConfig defaultConfig;    
  private final CrocodileSubsystem croc;
  private final RotatorSubsystem rotator;
  private final TelescopeSubsystem telescope;
  public BluePreloadPlusOneLeft(DrivebaseSubsystem driveBase, CrocodileSubsystem croc, RotatorSubsystem rotator, TelescopeSubsystem telescope) {
    // assign the drivebase and config file
    this.driveBase = driveBase;
    this.croc = croc;
    this.rotator = rotator;
    this.telescope = telescope;
    // sets the config variables to the speed and accel constants.
    this.defaultConfig = new TrajectoryConfig(Constants.AutoConstants.k_MAX_SPEED_MPS,
        Constants.AutoConstants.k_MAX_ACCEL_MPS_SQUARED);
    addCommands(
        // references the path file and sets the starting color and if the command is running first.
        // Close Gripper
        new InstantCommand(()-> croc.closeChomper()),
        // Arm Rotate 230
        new RotatorToPosition(rotator, telescope, 230),
        // Extend Telescope Out
        new TelescopeToPosition(telescope, 90),
        // Flex Wrist
        new InstantCommand(()-> croc.wristDown()),
        // Gripper Open
        new InstantCommand(()-> croc.openChomper()),
        // Wrist Straight
        // Retract Telescope
        // Arm Rotate 90
        new SwerveTrajectoryFollowCommand(driveBase, "preloadPlusOneLeft1", defaultConfig, true, true),
        // Close Gripper
        new SwerveTrajectoryFollowCommand(driveBase, "preloadPlusOneLeft2", defaultConfig, true, false));
        // Arm Rotate 135
        // Extend Telescope Out
        // Flex Wrist
        // Gripper Open
        // Wrist Straight
        // Retract Telescope
        // Arm Rotate 90
    // grabs any requirements needed for the drivebase from other running commands.
    addRequirements(driveBase);
  }
}
