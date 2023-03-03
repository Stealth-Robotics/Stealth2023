// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.RobotMap.Crocodile;
import frc.robot.subsystems.CrocodileSubsystem;
import frc.robot.subsystems.RotatorSubsystem;
import frc.robot.subsystems.TelescopeSubsystem;
import frc.robot.subsystems.Swerve.DrivebaseSubsystem;

public class BluePreloadParkCenter extends SequentialCommandGroup {
  //creates variables for the drivebase and defaultconfig.
  private final DrivebaseSubsystem driveBase;
  private final TrajectoryConfig defaultConfig;    
  private final CrocodileSubsystem croc;
  private final RotatorSubsystem rotator;
  private final TelescopeSubsystem telescope;
  public BluePreloadParkCenter(DrivebaseSubsystem driveBase, CrocodileSubsystem croc, RotatorSubsystem rotator, TelescopeSubsystem telescope) {
    //assign the drivebase and config file
    this.driveBase = driveBase;
    this.croc = croc;
    this.rotator = rotator;
    this.telescope = telescope;
    //sets the config variables to the speed and accel constants.
    this.defaultConfig = new TrajectoryConfig(Constants.AutoConstants.k_MAX_SPEED_MPS, Constants.AutoConstants.k_MAX_ACCEL_MPS_SQUARED);
    addCommands(
      //references the path file and sets the starting color and if the command is running first.
      /* Start Condition:
       - Wrist Straight
       - Cone In Gripper
       - Rotator At 180
       - Telescope Fully Retracted
      */

      //Command Group

      new InstantCommand(()-> croc.closeChomper()),
      new RotatorToPosition(rotator, telescope, 230),
      new TelescopeToPosition(telescope, 70000), //TODO: set to actual telescope position.
      new InstantCommand(()-> croc.wristDown()),
      new WaitCommand(0.2),
      new InstantCommand(()-> croc.openChomper()),
      new WaitCommand(.2),
      new InstantCommand(()-> croc.wristUp()),
      new WaitCommand(.2),
      //new TelescopeToPosition(telescope, 1000), //TODO: set to actual telescope position.
      new ResetTelescope(telescope),
      new RotatorToPosition(rotator, telescope, 90),
      new SwerveTrajectoryFollowCommand(driveBase,  "preloadParkCenter", defaultConfig, false, true)
      //LEVEL


    );
    //grabs any requirements needed for the drivebase from other running commands.
    addRequirements(driveBase);
  } 
}
