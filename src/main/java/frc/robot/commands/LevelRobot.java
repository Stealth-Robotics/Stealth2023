package frc.robot.commands;

import com.ctre.phoenix.sensors.WPI_Pigeon2;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Swerve.DrivebaseSubsystem;
public class LevelRobot extends CommandBase{
    private final DrivebaseSubsystem drive;
    private final PIDController pid = new PIDController(
        Constants.LevelRobotConstants.PID_kP, 
        Constants.LevelRobotConstants.PID_kI, 
        Constants.LevelRobotConstants.PID_kD);
    private final PIDController headingPid = new PIDController(
        Constants.LevelRobotConstants.HEADING_PID_kP,
        Constants.LevelRobotConstants.HEADING_PID_kI,
        Constants.LevelRobotConstants.HEADING_PID_kD);
    private double startingYaw;
    
    public LevelRobot(SwerveSubsystem drivetrain){
        drive = drivetrain;
        addRequirements(drive);
    }
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        pid.setTolerance(0);
        pid.setSetpoint(0);
        startingYaw = drive.getYawAsDouble() % 360;
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double roll = drive.getRoll();
        double yaw = drive.getYawAsDouble() % 360;
        double deltaYaw = yaw-startingYaw;
        double calculationMovement = pid.calculate(roll, 0);
        double calculationHeading = headingPid.calculate(deltaYaw * -1, 0);
        calculationMovement = MathUtil.clamp(calculationMovement, -Constants.LevelRobotConstants.LEVELING_DRIVE_SPEED_LIMIT, Constants.LevelRobotConstants.LEVELING_DRIVE_SPEED_LIMIT);
        calculationHeading = MathUtil.clamp(calculationHeading, -Constants.LevelRobotConstants.LEVELING_ROTATION_SPEED_LIMIT, Constants.LevelRobotConstants.LEVELING_ROTATION_SPEED_LIMIT);
        drive.drive(new Translation2d(calculationMovement, 0), 0, false, true);
        
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return pid.atSetpoint();
    }

}