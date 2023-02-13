package frc.robot.subsystems;

import frc.robot.SwerveModule;
import frc.robot.Constants;
import frc.robot.PhotonVisionCameraWrapper;
import frc.robot.RobotMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;

import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Swerve extends SubsystemBase {
    public PhotonVisionCameraWrapper pcw;

    public SwerveDrivePoseEstimator swerveOdometry;
    public SwerveModule[] mSwerveMods;
    public Pigeon2 gyro;

    private final PIDController xController = new PIDController(Constants.AutoConstants.X_CONTROLLER_P_COEFF, 0, 0);
    private final PIDController yController = new PIDController(Constants.AutoConstants.Y_CONTROLLER_P_COEFF, 0, 0);

    private final ProfiledPIDController thetaController = new ProfiledPIDController(
            Constants.AutoConstants.THETA_CONTROLLER_P_COEFF, 0, 0,
            Constants.AutoConstants.THETA_CONTROLLER_CONSTRAINTS);

    private final HolonomicDriveController pathController = new HolonomicDriveController(
            xController,
            yController,
            thetaController);

    private Field2d field2d;

    public Swerve() {
        pcw = new PhotonVisionCameraWrapper();

        gyro = new Pigeon2(RobotMap.Drivebase.PIGEON_ID);
        gyro.configFactoryDefault();
        zeroGyro();

        mSwerveMods = new SwerveModule[] {
            new SwerveModule(0, Constants.Swerve.Mod0.CONSTANTS),
            new SwerveModule(1, Constants.Swerve.Mod1.CONSTANTS),
            new SwerveModule(2, Constants.Swerve.Mod2.OFFSET),
            new SwerveModule(3, Constants.Swerve.Mod3.CONSTANTS)
        };
        //TODO: Set the actual pose
        swerveOdometry = new SwerveDrivePoseEstimator(Constants.Swerve.SWERVE_KINEMATICS, getYaw(), getModulePositions(), new Pose2d());

        field2d = new Field2d();
        SmartDashboard.putData(field2d);
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        SwerveModuleState[] swerveModuleStates =
            Constants.Swerve.SWERVE_KINEMATICS.toSwerveModuleStates(
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation, 
                                    getYaw()
                                )
                                : new ChassisSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation)
                                );
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.MAX_SPEED);

        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }
    }

    public void drive(ChassisSpeeds chassisSpeeds, boolean isOpenLoop)
    {
        SwerveModuleState[] swerveModuleStates = Constants.Swerve.SWERVE_KINEMATICS.toSwerveModuleStates(chassisSpeeds);

        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.MAX_SPEED);

        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }
    }

    public void drive(Trajectory.State targetState, Rotation2d targetRotation, boolean isOpenLoop) {
        // determine ChassisSpeeds from path state and positional feedback control from
        // HolonomicDriveController
        ChassisSpeeds targetChassisSpeeds = pathController.calculate(
                getPose(),
                targetState,
                targetRotation);
        // command robot to reach the target ChassisSpeeds
        drive(targetChassisSpeeds, isOpenLoop);
    }

    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.MAX_SPEED);
        
        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(desiredStates[mod.moduleNumber], false);
        }
    }    

    public Pose2d getPose() {
        return swerveOdometry.getEstimatedPosition();
    }

    public void resetOdometry(Pose2d pose) {
        swerveOdometry.resetPosition(getYaw(), getModulePositions(), pose);
    }

    public SwerveModuleState[] getModuleStates(){
        SwerveModuleState[] states = new SwerveModuleState[4];
        for(SwerveModule mod : mSwerveMods){
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }

    public SwerveModulePosition[] getModulePositions(){
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for(SwerveModule mod : mSwerveMods){
            positions[mod.moduleNumber] = mod.getPosition();
        }
        return positions;
    }

    public void zeroGyro(){
        gyro.setYaw(0);
        System.out.println("Hi " + gyro.getYaw());
    }
    public Rotation2d getYaw() {
        return (Constants.Swerve.INVERT_GYRO) ? Rotation2d.fromDegrees(360 - gyro.getYaw()) : Rotation2d.fromDegrees(gyro.getYaw());
    }

    public double getYawAsDouble() {
        return gyro.getYaw();
    }

    public double getPitchAsDouble() {
        return gyro.getPitch();
    }

    public double getRollAsDouble(){
        return gyro.getRoll();
    }

    @Override
    public void periodic(){

        swerveOdometry.update(getYaw(), getModulePositions());

        Optional<EstimatedRobotPose> result =
                pcw.getEstimatedGlobalPose(swerveOdometry.getEstimatedPosition());

        if(result.isPresent())
        {
            EstimatedRobotPose camPose = result.get();
            swerveOdometry.addVisionMeasurement(
                    camPose.estimatedPose.toPose2d(), 
                    camPose.timestampSeconds
            );
        }

        field2d.setRobotPose(getPose());
        
        for(SwerveModule mod : mSwerveMods){
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Cancoder", mod.getCanCoder().getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Integrated", mod.getPosition().angle.getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);    
        }
    }
}