package frc.robot.subsystems.Swerve;

import frc.robot.Constants;
import frc.robot.RobotMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DrivebaseSubsystem extends SubsystemBase {
    public SwerveDrivePoseEstimator swerveOdometry;
    public SwerveModule[] mSwerveMods;
    public Pigeon2 gyro;


    private final ProfiledPIDController thetaController = new ProfiledPIDController(
            Constants.AutoConstants.k_P_THETA_CONTROLLER, 0, 0,
            Constants.AutoConstants.k_THETA_CONTROLLER_CONSTRAINTS);

    private final PIDController xController = new PIDController(Constants.AutoConstants.k_PX_CONTROLLER, 0, 0);
    private final PIDController yController = new PIDController(Constants.AutoConstants.k_PY_CONTROLLER, 0, 0);

    private final HolonomicDriveController pathController = new HolonomicDriveController(
            xController,
            yController,
            thetaController);
    

    public DrivebaseSubsystem() {
        gyro = new Pigeon2(RobotMap.Pigeon.PIGEON_ID);
        gyro.configFactoryDefault();
        zeroGyro();

        mSwerveMods = new SwerveModule[] {
            new SwerveModule(0, Constants.DrivebaseConstants.MOD_0.constants),
            new SwerveModule(1, Constants.DrivebaseConstants.MOD_1.constants),
            new SwerveModule(2, Constants.DrivebaseConstants.MOD_2.constants),
            new SwerveModule(3, Constants.DrivebaseConstants.MOD_3.constants)
        };
        //TODO: Set the actual pose
        swerveOdometry = new SwerveDrivePoseEstimator(Constants.DrivebaseConstants.SWERVE_KINEMATICS, getYaw(), getModulePositions(), new Pose2d());
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        SwerveModuleState[] swerveModuleStates =
            Constants.DrivebaseConstants.SWERVE_KINEMATICS.toSwerveModuleStates(
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
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.DrivebaseConstants.MAX_SPEED);

        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }
    }    

        /**
     * Moves the drivebase around by running the swerve modules.
     * 
     * @param chassisSpeeds The x, y, and theta the drivebase must move in.
     */
    public void drive(ChassisSpeeds chassisSpeeds) {
    }

    public void drive(Trajectory.State targetState, Rotation2d targetRotation) {
        ChassisSpeeds targetChassisSpeeds = pathController.calculate(
                getPose(),
                targetState,
                targetRotation);
        // command robot to reach the target ChassisSpeeds
        drive(targetChassisSpeeds);
    }

    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.DrivebaseConstants.MAX_SPEED);
        
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
    }
    public Rotation2d getYaw() {
        return (Constants.DrivebaseConstants.INVERT_GYRO) ? Rotation2d.fromDegrees(360 - gyro.getYaw()) : Rotation2d.fromDegrees(gyro.getYaw());
    }

    public double getYawAsDouble(){
        return gyro.getYaw();
    }

    public double getPitch(){
        return gyro.getPitch();
    }

    public double getRoll(){
        return gyro.getRoll();
    }

    @Override
    public void periodic(){
        swerveOdometry.update(getYaw(), getModulePositions());  

        for(SwerveModule mod : mSwerveMods){
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Cancoder", mod.getCanCoder().getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Integrated", mod.getPosition().angle.getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);    
        }
    }
}