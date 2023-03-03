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

public class BluePreloadPlusOneRight extends SequentialCommandGroup {
  //creates variables for the drivebase and defaultconfig.
  private final DrivebaseSubsystem driveBase;
  private final TrajectoryConfig defaultConfig;    
  private final CrocodileSubsystem croc;
  private final RotatorSubsystem rotator;
  private final TelescopeSubsystem telescope;   
  public BluePreloadPlusOneRight(DrivebaseSubsystem driveBase, CrocodileSubsystem croc, RotatorSubsystem rotator, TelescopeSubsystem telescope) {
    //assign the drivebase and config file
    this.driveBase = driveBase;
    this.croc = croc;
    this.rotator = rotator;
    this.telescope = telescope;
    //sets the config variables to the speed and accel constants.
    this.defaultConfig = new TrajectoryConfig(Constants.AutoConstants.k_MAX_SPEED_MPS, Constants.AutoConstants.k_MAX_ACCEL_MPS_SQUARED);
    addCommands(
      //references the path file and sets the starting color and if the command is running first.
      // Close Gripper
      new InstantCommand(()-> croc.closeChomper()),
      // Arm Rotate 230
      new RotatorToPosition(rotator, telescope, 230),
      // Extend Telescope Out
      new TelescopeToPosition(telescope, 70000),
      // Flex Wrist
      new InstantCommand(()-> croc.wristDown()),
      // Gripper Open
      new InstantCommand(()-> croc.openChomper()),
      // Wrist Straight
      new InstantCommand(()-> croc.wristUp()),
      // Retract Telescope
      new ResetTelescope(telescope),
      // Arm Rotate 90
      new RotatorToPosition(rotator, telescope, 90),

      new SwerveTrajectoryFollowCommand(driveBase,  "preloadPlusOneRight1", defaultConfig, false, true),
      //Close Gripper
      new InstantCommand(()-> croc.closeChomper()),
      new SwerveTrajectoryFollowCommand(driveBase,  "preloadPlusOneRight2", defaultConfig, false, false),
      // Arm Rotate 135
      new RotatorToPosition(rotator, telescope, 230),
      // Extend Telescope Out
      new TelescopeToPosition(telescope, 70000),
      // Flex Wrist
      new InstantCommand(()-> croc.wristDown()),
      // Gripper Open
      new InstantCommand(()-> croc.openChomper()),
      // Wrist Straight
      new InstantCommand(()-> croc.wristUp()),
      // Retract Telescope
      new ResetTelescope(telescope),
      // Arm Rotate 90
      new RotatorToPosition(rotator, telescope, 90)
    );
    //grabs any requirements needed for the drivebase from other running commands.
    addRequirements(driveBase);
  } 
}
