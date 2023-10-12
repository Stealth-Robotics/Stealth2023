// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos;

import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.SharedConstants;
import frc.robot.commands.SwerveTrajectoryFollowCommand;
import frc.robot.subsystems.CrocodileSubsystem;
import frc.robot.subsystems.RotatorSubsystem;
import frc.robot.subsystems.TelescopeSubsystem;
import frc.robot.subsystems.Swerve.DrivebaseSubsystem;

public class EXIT_COMMUNITY extends SequentialCommandGroup {
  private final TrajectoryConfig defaultConfig;

  public EXIT_COMMUNITY(DrivebaseSubsystem driveBase, CrocodileSubsystem croc, RotatorSubsystem rotator,
      TelescopeSubsystem telescope) {
        this.defaultConfig = new TrajectoryConfig(SharedConstants.AutoConstants.k_MAX_SPEED_MPS,
        SharedConstants.AutoConstants.k_MAX_ACCEL_MPS_SQUARED);
    addCommands(
        new SwerveTrajectoryFollowCommand(driveBase, "moveOut", defaultConfig, true)

    /*
     * start
     * rotate telescope to scoring position
     * extend telescope to top position
     * move wrist to 90 degrees
     * drop cone using chomper
     * move wrist back up to original position
     * retract arm
     * rotate arm to standby/pickup position
     * move forward to the cone
     * extend arm out
     * move wrist down to grabbing position
     * move forward some more while wrist is sucking up cone to pick it up
     * retract arm in
     * move back to starting position
     * more over to the left/right in line up with poles
     * flip arm over
     * extend arm to top position
     * move wrist to 90 degrees
     * drop cone using chomper
     * move wrist back up to original position
     * retract arm back in
     * rotate arm to standby/pickup position
     * end
     */

    );
    // grabs any requirements needed for the drivebase from other running commands.
    addRequirements(driveBase);
  }
}
