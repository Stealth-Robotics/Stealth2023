// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos;

import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.SharedConstants;
import frc.robot.RobotMap.Crocodile;
import frc.robot.commands.ResetTelescope;
import frc.robot.commands.RotatorToPosition;
import frc.robot.commands.SwerveTrajectoryFollowCommand;
import frc.robot.commands.TelescopeToPosition;
import frc.robot.commands.Presets.HighPresetSequence;
import frc.robot.commands.Presets.PickupPresetSequence;
import frc.robot.commands.Presets.StowPresetSequence;
import frc.robot.subsystems.CrocodileSubsystem;
import frc.robot.subsystems.RotatorSubsystem;
import frc.robot.subsystems.TelescopeSubsystem;
import frc.robot.subsystems.Swerve.DrivebaseSubsystem;

public class PreloadPlusOneLeft extends SequentialCommandGroup {
  // creates variables for the drivebase and defaultconfig.
  private final DrivebaseSubsystem driveBase;
  private final TrajectoryConfig defaultConfig;
  private final CrocodileSubsystem croc;
  private final RotatorSubsystem rotator;
  private final TelescopeSubsystem telescope;

  public PreloadPlusOneLeft(DrivebaseSubsystem driveBase, CrocodileSubsystem croc, RotatorSubsystem rotator,
      TelescopeSubsystem telescope) {
    // assign the drivebase and config file
    this.driveBase = driveBase;
    this.croc = croc;
    this.rotator = rotator;
    this.telescope = telescope;
    // sets the config variables to the speed and accel constants.
    this.defaultConfig = new TrajectoryConfig(SharedConstants.AutoConstants.k_MAX_SPEED_MPS,
        SharedConstants.AutoConstants.k_MAX_ACCEL_MPS_SQUARED);
    addCommands(
        new HighPresetSequence(telescope, rotator, croc, null, () -> true),
        new InstantCommand(()->croc.setIntakeSpeed(-1)),
        new WaitCommand(0.25),
        new InstantCommand(()->croc.setIntakeSpeed(0)),
        new ParallelCommandGroup(
          new PickupPresetSequence(telescope, rotator, croc, null, () -> true).withTimeout(3),
          new SwerveTrajectoryFollowCommand(driveBase, "preloadPlusOneLeft1", defaultConfig, true)
        ),
        new StowPresetSequence(telescope, rotator, croc),
        new SwerveTrajectoryFollowCommand(driveBase, "preloadPlusOneLeft2", defaultConfig),
        new HighPresetSequence(telescope, rotator, croc, null, () -> true),
        new InstantCommand(()->croc.setIntakeSpeed(-1)),
        new WaitCommand(0.25),
        new StowPresetSequence(telescope, rotator, croc)

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
