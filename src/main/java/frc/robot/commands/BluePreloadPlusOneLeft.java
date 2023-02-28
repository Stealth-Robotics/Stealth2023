// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve.DrivebaseSubsystem;

public class BluePreloadPlusOneLeft extends SequentialCommandGroup {
  // creates variables for the drivebase and defaultconfig.
  private final DrivebaseSubsystem driveBase;
  private final TrajectoryConfig defaultConfig;

  public BluePreloadPlusOneLeft(DrivebaseSubsystem driveBase) {
    // assign the drivebase and config file
    this.driveBase = driveBase;
    // sets the config variables to the speed and accel constants.
    this.defaultConfig = new TrajectoryConfig(Constants.AutoConstants.k_MAX_SPEED_MPS,
        Constants.AutoConstants.k_MAX_ACCEL_MPS_SQUARED);
    addCommands(
        // references the path file and sets the starting color and if the command is
        // running first.
        // Close Gripper
        // Arm Rotate 135
        // Extend Telescope Out
        // Flex Wrist
        // Gripper Open
        // Wrist Straight
        // Retract Telescope
        // Arm Rotate 90
        new SwerveTrajectoryFollowCommand(driveBase, "preloadPlusOneLeft1", defaultConfig, true, true),
        new SwerveTrajectoryFollowCommand(driveBase, "preloadPlusOneLeft2", defaultConfig, true, false));
    // grabs any requirements needed for the drivebase from other running commands.
    addRequirements(driveBase);
  }
}
