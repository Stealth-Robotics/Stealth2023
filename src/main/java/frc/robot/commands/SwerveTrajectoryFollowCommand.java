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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Swerve.DrivebaseSubsystem;

/**
 * Custom implementation of
 * {@link edu.wpi.first.wpilibj2.command.SwerveControllerCommand
 * SwerveControllerCommand}
 * to simplify construction, allow PathPlanner paths, and enable custom logging
 */
public class SwerveTrajectoryFollowCommand extends CommandBase {

    private final DrivebaseSubsystem drivetrain;
    private PathPlannerTrajectory trajectory;
    private Timer timer = new Timer();
    private Pose2d initialPathPlannerPose = null;
    private boolean isInitial;

    public SwerveTrajectoryFollowCommand(DrivebaseSubsystem drivetrain, String pathFilename, TrajectoryConfig config,
            boolean isReversed, boolean isInitial) {
        this.drivetrain = drivetrain;
        this.isInitial = isInitial;
        addRequirements(drivetrain);

        PathPlannerTrajectory ppTrajectory = PathPlanner.loadPath(pathFilename, config.getMaxVelocity(),
                config.getMaxAcceleration(), isReversed);
        this.trajectory = ppTrajectory;
    }

    public SwerveTrajectoryFollowCommand(DrivebaseSubsystem drivetrain, String pathFilename, TrajectoryConfig config,
            boolean isInitial) {
        this(drivetrain, pathFilename, config, false, isInitial);
    }

    public SwerveTrajectoryFollowCommand(DrivebaseSubsystem drivetrain, String pathFilename, TrajectoryConfig config) {
        this(drivetrain, pathFilename, config, false, false);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {

        trajectory = PathPlannerTrajectory.transformTrajectoryForAlliance(trajectory, DriverStation.getAlliance());

        if (isInitial) {
            initialPathPlannerPose = new Pose2d(
                    trajectory.getInitialPose().getTranslation(),
                    trajectory.getInitialState().holonomicRotation);
        }

        if (initialPathPlannerPose != null) {
            drivetrain.resetOdometry(initialPathPlannerPose);
        }

        timer.reset();
        timer.start();
        // trajectories
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double currentTime = timer.get();
        // The target state of the trajectory right now (the robot's pose and velocity)
        Trajectory.State targetState = trajectory.sample(currentTime);
        Rotation2d targetRotation = targetState.poseMeters.getRotation();
        // Check if we are using PathPlanner trajectories
        if (trajectory instanceof PathPlannerTrajectory) {
            targetState = ((PathPlannerTrajectory) trajectory).sample(currentTime);
            targetRotation = ((PathPlannerState) targetState).holonomicRotation;
        }

        drivetrain.drive(targetState, targetRotation, true);

        // Pose2d targetPose = targetState.poseMeters;
        // SmartDashboard.putNumber("Target Heading",
        // targetPose.getRotation().getDegrees());
        // SmartDashboard.putNumber("Target X", targetPose.getX());
        // SmartDashboard.putNumber("Target Y", targetPose.getY());
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        // Default constructor of chassisSpeeds is 0 on everything
        if (interrupted)
            drivetrain.drive(new ChassisSpeeds(), true);
        timer.stop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        // the path is time parametrized and takes a certain number of seconds
        return timer.hasElapsed(trajectory.getTotalTimeSeconds());
    }
}