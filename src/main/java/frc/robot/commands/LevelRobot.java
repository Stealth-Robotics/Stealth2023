package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Swerve.DrivebaseSubsystem;

public class LevelRobot extends CommandBase{
    private final DrivebaseSubsystem drive;
    private final PIDController pid = new PIDController(0.1, 0.001, 0.005);
    private final PIDController headingPid = new PIDController(0.04, 0, 0.0025);
    private double startingYaw;
    
    public LevelRobot(DrivebaseSubsystem drivetrain){
        drive = drivetrain;
        addRequirements(drive);
    }
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        pid.setTolerance(0);
        pid.setSetpoint(0);
        startingYaw = drive.getYawAsDouble() % 360;
        System.out.println("Command Init");
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double roll = drive.getRollAsDouble();
        double yaw = drive.getYawAsDouble() % 360;
        double deltaYaw = yaw-startingYaw;
        double calculationMovement = pid.calculate(roll, 0);
        double calculationHeading = headingPid.calculate(deltaYaw * -1, 0);
        calculationMovement = MathUtil.clamp(calculationMovement, -0.45, 0.45);
        calculationHeading = MathUtil.clamp(calculationHeading, -0.5, 0.5);
        drive.drive(new Translation2d(calculationMovement, 0), 0, false, true);
        System.out.printf("Calculation: %5.2f | Gyro: %5.2f | CalculationHeading: %5.2f | GyroYaw: %5.2f | HeadingError: %5.2f\n",calculationMovement, roll, calculationHeading, yaw, deltaYaw);
        
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        //return pid.atSetpoint() && pid;
        return false;
    }

}