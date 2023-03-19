// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Swerve.DrivebaseSubsystem;

/**
 * Custom implementation of
 * {@link edu.wpi.first.wpilibj2.command.SwerveControllerCommand
 * SwerveControllerCommand}
 * to simplify construction, allow PathPlanner paths, and enable custom logging
 */
public class ReverseSwerveTrajectoryFollowCommand extends SwerveTrajectoryFollowCommand {

    public ReverseSwerveTrajectoryFollowCommand(DrivebaseSubsystem drivetrain, String pathFilename, TrajectoryConfig config,
    boolean isInitial) {
        super(drivetrain, pathFilename, config, true, isInitial);
    }


}