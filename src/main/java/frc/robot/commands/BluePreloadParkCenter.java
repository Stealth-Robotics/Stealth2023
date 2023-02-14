// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

public class BluePreloadParkCenter extends SequentialCommandGroup {
  //creates variables for the drivebase and defaultconfig.
  private final Swerve driveBase;
  private final TrajectoryConfig defaultConfig;    
  public BluePreloadParkCenter(Swerve driveBase) {
    //assign the drivebase and config file
    this.driveBase = driveBase;
    //sets the config variables to the speed and accel constants.
    this.defaultConfig = new TrajectoryConfig(Constants.AutoConstants.MAX_SPEED_METERS_PER_SEC, Constants.AutoConstants.MAX_ACCEL_METERS_PER_SEC_SQUARED);
    addCommands(
      //references the path file and sets the starting color and if the command is running first.
      new SwerveControllerFollower(driveBase,  "preloadParkCenter", defaultConfig, false, true)
    );
    //grabs any requirements needed for the drivebase from other running commands.
    addRequirements(driveBase);
  } 
}
