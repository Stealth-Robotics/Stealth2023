package frc.robot;

import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.SensorTimeBase;

public final class CTREConfigs {
    public TalonFXConfiguration swerveAngleFXConfig;
    public TalonFXConfiguration swerveDriveFXConfig;
    public CANCoderConfiguration swerveCanCoderConfig;

    public CTREConfigs() {
        swerveAngleFXConfig = new TalonFXConfiguration();
        swerveDriveFXConfig = new TalonFXConfiguration();
        swerveCanCoderConfig = new CANCoderConfiguration();

        /* Swerve Angle Motor Configurations */
        SupplyCurrentLimitConfiguration angleSupplyLimit = new SupplyCurrentLimitConfiguration(
                SharedConstants.DrivebaseConstants.ANGLE_ENABLE_CURRENT_LIMIT,
                SharedConstants.DrivebaseConstants.ANGLE_CONT_CURRENT_LIMIT,
                SharedConstants.DrivebaseConstants.ANGLE_PEAK_CURRENT_LIMIT,
                SharedConstants.DrivebaseConstants.ANGLE_PEAK_CURRENT_DURATION);

        swerveAngleFXConfig.slot0.kP = SharedConstants.DrivebaseConstants.angleKP;
        swerveAngleFXConfig.slot0.kI = SharedConstants.DrivebaseConstants.angleKI;
        swerveAngleFXConfig.slot0.kD = SharedConstants.DrivebaseConstants.angleKD;
        swerveAngleFXConfig.slot0.kF = SharedConstants.DrivebaseConstants.angleKF;
        swerveAngleFXConfig.supplyCurrLimit = angleSupplyLimit;

        /* Swerve Drive Motor Configuration */
        SupplyCurrentLimitConfiguration driveSupplyLimit = new SupplyCurrentLimitConfiguration(
                SharedConstants.DrivebaseConstants.DRIVE_ENABLE_CURRENT_LIMIT,
                SharedConstants.DrivebaseConstants.DRIVE_CONT_CURRENT_LIMIT,
                SharedConstants.DrivebaseConstants.DRIVE_PEAK_CURRENT_LIMIT,
                SharedConstants.DrivebaseConstants.DRIVE_PEAK_CURRENT_DURATION);

        swerveDriveFXConfig.slot0.kP = SharedConstants.DrivebaseConstants.driveKP;
        swerveDriveFXConfig.slot0.kI = SharedConstants.DrivebaseConstants.driveKI;
        swerveDriveFXConfig.slot0.kD = SharedConstants.DrivebaseConstants.driveKD;
        swerveDriveFXConfig.slot0.kF = SharedConstants.DrivebaseConstants.driveKF;
        swerveDriveFXConfig.supplyCurrLimit = driveSupplyLimit;
        swerveDriveFXConfig.openloopRamp = SharedConstants.DrivebaseConstants.OPEN_LOOP_RAMP;
        swerveDriveFXConfig.closedloopRamp = SharedConstants.DrivebaseConstants.CLOSED_LOOP_RAMP;

        /* Swerve CANCoder Configuration */
        swerveCanCoderConfig.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
        swerveCanCoderConfig.sensorDirection = SharedConstants.DrivebaseConstants.CAN_CODER_INVERT;
        swerveCanCoderConfig.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
        swerveCanCoderConfig.sensorTimeBase = SensorTimeBase.PerSecond;
    }
}