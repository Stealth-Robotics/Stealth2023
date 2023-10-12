// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.CrocodileSubsystem;
import frc.robot.subsystems.RotatorSubsystem;
import frc.robot.subsystems.TelescopeSubsystem;
import frc.robot.subsystems.Swerve.DrivebaseSubsystem;

public class DO_NOTHING extends SequentialCommandGroup {

  public DO_NOTHING(DrivebaseSubsystem driveBase, CrocodileSubsystem croc, RotatorSubsystem rotator,
      TelescopeSubsystem telescope) {
    addCommands(
        new InstantCommand()

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
