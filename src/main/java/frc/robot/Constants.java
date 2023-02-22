package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.lib.util.COTSFalconSwerveConstants;
import frc.lib.util.SwerveModuleConstants;
import java.io.IOException;

import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;


public final class Constants {
    public static final double stickDeadband = 0.1;
    public static final class IOConstants{
        public static final int kDriverControllerPort = 0;
        public static final int kOperatorControllerPort = 1;
        public static final int controlBoardPort = 2;
    }
    public static final class Swerve {
        public static final int pigeonID = 1;
        public static final boolean invertGyro = false; // Always ensure Gyro is CCW+ CW-
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
    public static final class TeleopConstants{
        public static final double SITCK_DEADBAND = 0.1;
    }
    public static final class IOConstants{
        public static final int k_DRIVER_CONTROLLER_PORT = 0;
        public static final int k_OPERATOR_CONTROLLER_PORT = 1;
    }
    public static final class DrivebaseConstants {
        public static final int PIGEON_ID = 1;
        public static final boolean INVERT_GYRO = false; // Always ensure Gyro is CCW+ CW-

        public static final COTSFalconSwerveConstants CHOSEN_MODULE =  
            COTSFalconSwerveConstants.SDSMK4(COTSFalconSwerveConstants.driveGearRatios.SDSMK4_L2);

        /* Drivetrain Constants */
        public static final double TRACKWIDTH = Units.inchesToMeters(17.5); 
        public static final double WHEELBASE = Units.inchesToMeters(26.75); 
        public static final double WHEEL_CIRCUMFERENCE = CHOSEN_MODULE.wheelCircumference;

        /* Swerve Kinematics 
         * No need to ever change this unless you are not doing a traditional rectangular/square 4 module swerve */
         public static final SwerveDriveKinematics SWERVE_KINEMATICS = new SwerveDriveKinematics(
            new Translation2d(WHEELBASE / 2.0, TRACKWIDTH / 2.0),
            new Translation2d(WHEELBASE / 2.0, -TRACKWIDTH / 2.0),
            new Translation2d(-WHEELBASE / 2.0, TRACKWIDTH / 2.0),
            new Translation2d(-WHEELBASE / 2.0, -TRACKWIDTH / 2.0));

        /* Module Gear Ratios */
        public static final double DRIVE_GEAR_RATIO = CHOSEN_MODULE.driveGearRatio;
        public static final double ANGLE_GEAR_RATIO = CHOSEN_MODULE.angleGearRatio;

        /* Motor Inverts */
        public static final boolean ANGLE_MOTOR_INVERT = CHOSEN_MODULE.angleMotorInvert;
        public static final boolean DRIVE_MOTOR_INVERT = CHOSEN_MODULE.driveMotorInvert;

        /* Angle Encoder Invert */
        public static final boolean CAN_CODER_INVERT = CHOSEN_MODULE.canCoderInvert;

        /* Swerve Current Limiting */
        //TODO: These values must be tuned for this robot
        //TEST ON ROBOT
        public static final int ANGLE_CONT_CURRENT_LIMIT = 25;
        public static final int ANGLE_PEAK_CURRENT_LIMIT = 40;
        public static final double ANGLE_PEAK_CURRENT_DURATION = 0.1;
        public static final boolean ANGLE_ENABLE_CURRENT_LIMIT = true;

        public static final int DRIVE_CONT_CURRENT_LIMIT = 35;
        public static final int DRIVE_PEAK_CURRENT_LIMIT = 60;
        public static final double DRIVE_PEAK_CURRENT_DURATION = 0.1;
        public static final boolean DRIVE_ENABLE_CURRENT_LIMIT = true;

        /* These values are used by the drive falcon to ramp in open loop and closed loop driving.
         * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc */
        public static final double OPEN_LOOP_RAMP = 0.25;
        public static final double CLOSED_LOOP_RAMP = 0.0;

        /* Angle Motor PID Values */
        public static final double angleKP = CHOSEN_MODULE.angleKP;
        public static final double angleKI = CHOSEN_MODULE.angleKI;
        public static final double angleKD = CHOSEN_MODULE.angleKD;
        public static final double angleKF = CHOSEN_MODULE.angleKF;

        /* Drive Motor PID Values */
        public static final double driveKP = 0.05; //TODO: This must be tuned to specific robot
        public static final double driveKI = 0.0;
        public static final double driveKD = 0.0;
        public static final double driveKF = 0.0;

        /* Drive Motor Characterization Values 
         * Divide SYSID values by 12 to convert from volts to percent output for CTRE */
        public static final double driveKS = (0.32 / 12); //TODO: This must be tuned to specific robot
        public static final double driveKV = (1.51 / 12);
        public static final double driveKA = (0.27 / 12);

        /* Swerve Profiling Values */
        /** Meters per Second */
        public static final double MAX_SPEED = 4.5; //TODO: This must be tuned to specific robot
        /** Radians per Second */
        public static final double MAX_ANGULAR_VELOCITY = 10.0; //TODO: This must be tuned to specific robot

        /* Slowmode multipliers */
        public static final double slowmodeMultiplier = 0.5; //TODO: Modify for drivers preference

        /* Neutral Modes */
        public static final NeutralMode ANGLE_NEUTRAL_MODE = NeutralMode.Coast;
        public static final NeutralMode DRIVE_NEUTRAL_MODE = NeutralMode.Brake;
        
        /* Module Specific Constants */
        /* Front Left Module - Module 0 */
        public static final class MOD_0 { //TODO: This must be tuned to specific robot
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(292.840);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(
                    RobotMap.Drivebase.Mod0.DRIVE_MOTOR_ID, 
                    RobotMap.Drivebase.Mod0.ANGLE_MOTOR_ID, 
                    RobotMap.Drivebase.Mod0.CANCODER_ID, 
                    angleOffset
                );
        }
        

        /* Front Right Module - Module 1 */
        public static final class MOD_1 { //TODO: This must be tuned to specific robot
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(248.936);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(
                    RobotMap.Drivebase.Mod1.DRIVE_MOTOR_ID, 
                    RobotMap.Drivebase.Mod1.ANGLE_MOTOR_ID, 
                    RobotMap.Drivebase.Mod1.CANCODER_ID, 
                    angleOffset
                );
        }
        
        /* Back Left Module - Module 2 */
        public static final class MOD_2 { //TODO: This must be tuned to specific robot
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(178.651);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(
                    RobotMap.Drivebase.Mod2.DRIVE_MOTOR_ID, 
                    RobotMap.Drivebase.Mod2.ANGLE_MOTOR_ID, 
                    RobotMap.Drivebase.Mod2.CANCODER_ID, 
                    angleOffset
                );
        }

        /* Back Right Module - Module 3 */
        public static final class MOD_3 { //TODO: This must be tuned to specific robot
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(75.997);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(
                    RobotMap.Drivebase.Mod3.DRIVE_MOTOR_ID, 
                    RobotMap.Drivebase.Mod3.ANGLE_MOTOR_ID, 
                    RobotMap.Drivebase.Mod3.CANCODER_ID, 
                    angleOffset
                );
        }
    }

    public static final class AutoConstants { //TODO: The below constants are used in the example auto, and must be tuned to specific robot
        public static final double k_MAX_SPEED_MPS = 3;
        public static final double k_MAX_ACCEL_MPS_SQUARED = 3;
        public static final double k_MAX_ANGULAR_SPEED_RADS_PER_SEC = Math.PI;
        public static final double k_MAX_ANGULAR_ACCEL_RADS_PER_SEC_SQUARED = Math.PI;
    
        public static final double k_PX_CONTROLLER = 1;
        public static final double k_PY_CONTROLLER = 1;
        public static final double k_P_THETA_CONTROLLER = 1;
    
        /* Constraint for the motion profilied robot angle controller */
        public static final TrapezoidProfile.Constraints k_THETA_CONTROLLER_CONSTRAINTS =
            new TrapezoidProfile.Constraints(
                k_MAX_ANGULAR_SPEED_RADS_PER_SEC, k_MAX_ANGULAR_ACCEL_RADS_PER_SEC_SQUARED);
    }
    public static final class ScoringPositions{
        //FOR TESTING
        public static final Pose2d left = new Pose2d(0.0, 0.0, new Rotation2d(0));
        public static final Pose2d middle = new Pose2d(Units.feetToMeters(1.0), 0.0, Rotation2d.fromDegrees(0.0));
        public static final Pose2d right = new Pose2d(Units.feetToMeters(2.0), 0.0, Rotation2d.fromDegrees(0.0));


        //REAL SCORING POSITIONS, numbers get larger as moving right
        public static final Pose2d BLUE_CONE_1 = new Pose2d(1.5, 0.5, new Rotation2d(0));
        public static final Pose2d BLUE_CONE_2 = new Pose2d(1.5, 1.65, new Rotation2d(0));
        public static final Pose2d BLUE_CONE_3 = new Pose2d(1.5, 2.2, new Rotation2d(0));
        public static final Pose2d BLUE_CONE_4 = new Pose2d(1.5, 3.3, new Rotation2d(0));
        public static final Pose2d BLUE_CONE_5 = new Pose2d(1.5, 3.85, new Rotation2d(0));
        public static final Pose2d BLUE_CONE_6 = new Pose2d(1.5, 5.0, new Rotation2d(0));
        
        public static final Pose2d BLUE_CUBE_1 = new Pose2d(1.5, 1.05, new Rotation2d(0));
        public static final Pose2d BLUE_CUBE_2 = new Pose2d(1.5, 2.75, new Rotation2d(0));
        public static final Pose2d BLUE_CUBE_3 = new Pose2d(1.5, 4.40, new Rotation2d(0));

        public static final Pose2d RED_CONE_1 = new Pose2d(15.1, 0.5, new Rotation2d(0));
        public static final Pose2d RED_CONE_2 = new Pose2d(15.1, 1.65, new Rotation2d(0));
        public static final Pose2d RED_CONE_3 = new Pose2d(15.1, 2.2, new Rotation2d(0));
        public static final Pose2d RED_CONE_4 = new Pose2d(15.1, 3.3, new Rotation2d(0));
        public static final Pose2d RED_CONE_5 = new Pose2d(15.1, 3.85, new Rotation2d(0));
        public static final Pose2d RED_CONE_6 = new Pose2d(15.1, 5.0, new Rotation2d(0));

        public static final Pose2d RED_CUBE_1 = new Pose2d(15.1, 1.05, new Rotation2d(0));
        public static final Pose2d RED_CUBE_2 = new Pose2d(15.1, 2.75, new Rotation2d(0));
        public static final Pose2d RED_CUBE_3 = new Pose2d(15.1, 4.40, new Rotation2d(0));

    }
}
