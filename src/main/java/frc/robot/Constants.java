package frc.robot;

import java.io.IOException;

import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.lib.util.COTSFalconSwerveConstants;
import frc.lib.util.SwerveModuleConstants;


public final class Constants {
    public static final double STICK_DEADBAND = 0.1;

    public static final class IOConstants {
        public static final int DRIVER_CONTROLLER_PORT = 0;
        public static final int OPERATOR_CONTROLLER_PORT = 1;
    }

    public static final class PhotonVisionConstants {
        public final static String CAMERA_NAME = "photonvision";

        // TODO: Enter actual values (In meters and degrees)
        public static final Transform3d ROBOT_CENTER_TO_CAMERA = new Transform3d(
            new Translation3d(0.0, 0.0, 0.0),
            new Rotation3d(0, 0, 0)
        );

        public final static AprilTagFieldLayout APRIL_TAG_FIELD_LAYOUT;

        static {
            try {
                APRIL_TAG_FIELD_LAYOUT = AprilTagFields.k2023ChargedUp.loadAprilTagLayoutField();
            } catch (IOException e) {
                throw new RuntimeException("I/O exception with april tag field layout", e);
            }
        }

        // TODO: Choose Pose Strategy
        public final static PoseStrategy POSE_STRATEGY = PoseStrategy.LOWEST_AMBIGUITY;
    }


    public static final class Swerve {
        public static final boolean INVERT_GYRO = false; // Always ensure Gyro is CCW+ CW-

        public static final COTSFalconSwerveConstants CHOSEN_MODULE = COTSFalconSwerveConstants
                .SDSMK4(COTSFalconSwerveConstants.driveGearRatios.SDSMK4_L2);

        /* Drivetrain Constants */
        public static final double TRACK_WIDTH = Units.inchesToMeters(17.5);
        public static final double WHEEL_BASE = Units.inchesToMeters(26.75);
        public static final double WHEEL_CIRCUMFRENCE = CHOSEN_MODULE.wheelCircumference;

        /*
         * Swerve Kinematics
         * No need to ever change this unless you are not doing a traditional
         * rectangular/square 4 module swerve
         */
        public static final SwerveDriveKinematics SWERVE_KINEMATICS = new SwerveDriveKinematics(
                new Translation2d(WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0),
                new Translation2d(WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0),
                new Translation2d(-WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0),
                new Translation2d(-WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0));

        /* Module Gear Ratios */
        public static final double DRIVE_GEAR_RATIO = CHOSEN_MODULE.driveGearRatio;
        public static final double ANGLE_GEAR_RATIO = CHOSEN_MODULE.angleGearRatio;

        /* Motor Inverts */
        public static final boolean ANGLE_MOTOR_INVERT = CHOSEN_MODULE.angleMotorInvert;
        public static final boolean DRIVE_MOTOR_INVERT = CHOSEN_MODULE.driveMotorInvert;

        /* Angle Encoder Invert */
        public static final boolean CANCODER_INVERT = CHOSEN_MODULE.canCoderInvert;

        /* Swerve Current Limiting */
        public static final int ANGLE_CONTINUOUS_CURRENT_LIMIT = 25;
        public static final int ANGLE_PEAK_CURRENT_LIMIT = 40;
        public static final double ANGLE_PEAK_CURRENT_DURATION = 0.1;
        public static final boolean ANGLE_ENABLE_CURRENT_LIMIT = true;

        public static final int DRIVE_CONTINUOUS_CURRENT_LIMIT = 35;
        public static final int DRIVE_PEAK_CURRENT_LIMIT = 60;
        public static final double DRIVE_PEAK_CURRENT_DURATION = 0.1;
        public static final boolean DRIVE_ENABLE_CURRENT_LIMIT = true;

        /*
         * These values are used by the drive falcon to ramp in open loop and closed
         * loop driving.
         * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc
         */
        public static final double OPEN_LOOP_RAMP = 0.25;
        public static final double CLOSED_LOOP_RAMP = 0.0;

        /* Angle Motor PID Values */
        public static final double ANGLE_P_COEFF = CHOSEN_MODULE.angleKP;
        public static final double ANGLE_I_COEFF = CHOSEN_MODULE.angleKI;
        public static final double ANGLE_D_COEFF = CHOSEN_MODULE.angleKD;
        public static final double ANGLE_F_COEFF = CHOSEN_MODULE.angleKF;

        /* Drive Motor PID Values */
        public static final double DRIVE_P_COEFF = 0.05; // TODO: This must be tuned to specific robot
        public static final double DRIVE_I_COEFF = 0.0;
        public static final double DRIVE_D_COEFF = 0.0;
        public static final double DRIVE_F_COEFF = 0.0;

        /*
         * Drive Motor Characterization Values
         * Divide SYSID values by 12 to convert from volts to percent output for CTRE
         */
        public static final double DRIVE_KS = (0.32 / 12); // TODO: This must be tuned to specific robot
        public static final double DRIVE_KV = (1.51 / 12);
        public static final double DRIVE_KA = (0.27 / 12);

        /* Swerve Profiling Values */
        /** Meters per Second */
        public static final double MAX_SPEED = 4.5; // TODO: This must be tuned to specific robot
        /** Radians per Second */
        public static final double MAX_ANGULAR_VELOCITY = 10.0; // TODO: This must be tuned to specific robot

        /* Neutral Modes */
        public static final NeutralMode ANGLE_NEUTRAL_MODE = NeutralMode.Coast;
        public static final NeutralMode DRIVE_NEUTRAL_MODE = NeutralMode.Brake;

        /* Module Specific Constants */
        /* Front Left Module - Module 0 */
        public static final class Mod0 { // TODO: This must be tuned to specific robot
            public static final Rotation2d ANGLE_OFFSET = Rotation2d.fromDegrees(112.84);
            public static final SwerveModuleConstants CONSTANTS = new SwerveModuleConstants(
                    RobotMap.Drivebase.Mod0.DRIVE_MOTOR_ID,
                    RobotMap.Drivebase.Mod0.ANGLE_MOTOR_ID,
                    RobotMap.Drivebase.Mod0.CANCODER_ID,
                    ANGLE_OFFSET);
        }

        /* Front Right Module - Module 1 */
        public static final class Mod1 { // TODO: This must be tuned to specific robot
            public static final Rotation2d ANGLE_OFFSET = Rotation2d.fromDegrees(68.936);
            public static final SwerveModuleConstants CONSTANTS = new SwerveModuleConstants(
                    RobotMap.Drivebase.Mod1.DRIVE_MOTOR_ID,
                    RobotMap.Drivebase.Mod1.ANGLE_MOTOR_ID,
                    RobotMap.Drivebase.Mod1.CANCODER_ID,
                    ANGLE_OFFSET);
        }

        /* Back Left Module - Module 2 */
        public static final class Mod2 { // TODO: This must be tuned to specific robot
            public static final Rotation2d ANGLE_OFFSET = Rotation2d.fromDegrees(75.434);
            public static final SwerveModuleConstants OFFSET = new SwerveModuleConstants(
                    RobotMap.Drivebase.Mod2.DRIVE_MOTOR_ID,
                    RobotMap.Drivebase.Mod2.ANGLE_MOTOR_ID,
                    RobotMap.Drivebase.Mod2.CANCODER_ID,
                    ANGLE_OFFSET);
        }

        /* Back Right Module - Module 3 */
        public static final class Mod3 { // TODO: This must be tuned to specific robot
            public static final Rotation2d ANGLE_OFFSET = Rotation2d.fromDegrees(-104.003);
            public static final SwerveModuleConstants CONSTANTS = new SwerveModuleConstants(
                    RobotMap.Drivebase.Mod3.DRIVE_MOTOR_ID,
                    RobotMap.Drivebase.Mod3.ANGLE_MOTOR_ID,
                    RobotMap.Drivebase.Mod3.CANCODER_ID,
                    ANGLE_OFFSET);
        }
    }

    public static final class AutoConstants {
        public static final double MAX_SPEED_METERS_PER_SEC = 3;
        public static final double MAX_ACCEL_METERS_PER_SEC_SQUARED = 3;
        public static final double MAX_ANGULAR_SPEED_RAD_PER_SEC = Math.PI;
        public static final double MAX_ANGULAR_SPEED_RAD_PER_SEC_SQUARED = Math.PI;

        // TODO: Tune - Current Values From FRC 4089 2022
        public static final double X_CONTROLLER_P_COEFF = 3;
        public static final double Y_CONTROLLER_P_COEFF = 3;
        public static final double THETA_CONTROLLER_P_COEFF = 4;

        /* Constraint for the motion profilied robot angle controller */
        public static final TrapezoidProfile.Constraints THETA_CONTROLLER_CONSTRAINTS = new TrapezoidProfile.Constraints(
                MAX_ANGULAR_SPEED_RAD_PER_SEC, MAX_ANGULAR_SPEED_RAD_PER_SEC_SQUARED);
    }

    


}
