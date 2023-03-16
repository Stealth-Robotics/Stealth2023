package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
/*
 * DIAGRAM OF ROTATOR CORDINATE SYSTEM
 * 
 *        +180
 *          |
 *          |
 *          |
 *  +270 ---0----- +90 (FRONT OF ROBOT)
 */

public class RotatorSubsystem extends SubsystemBase {
    private final WPI_TalonFX rotationMotorA;
    private final WPI_TalonFX rotationMotorB;
    // Radians Per Second
    private static final double MAX_VELOCITY = 3.0;
    // Radians Per Second Squared
    private static final double MAX_ACCELERATION = 10.0;
    // PID Constants
    private static final double ROTATOR_P_COEFF = 0.75;
    private static final double ROTATOR_I_COEFF = 0;
    private static final double ROTATOR_D_COEFF = 0;
    // Feedforward Constants
    private static final double ROTATOR_KS_COEFF = 0;
    private static final double ROTATOR_KG_COEFF = 0.12;
    // Volt Second Per Rad
    private static final double ROTATOR_KV_COEFF = 0;
    // Volt Second Squared Per Rad
    private static final double ROTATOR_KA_COEFF = 0;
    // Offset of the encoder. See diagram above for reference
    private static final double ENCODER_OFFSET = 138;
    // Bounds of the rotator, degrees
    private static final double HIGH_BOUND = 285;
    private static final double LOW_BOUND = 70;
    // Speed Multiplier
    private static final double ROTATOR_SPEED_MULTIPLIER = 1.0;

    private final PIDController pid;
    private final ArmFeedforward feedforward;
    // Absolute encoder
    private final DutyCycleEncoder encoder;
    // Wether or not to log data in periodic
    private boolean log = false;
    // The speed limit of the rotator
    // This is for safety of people and robot
    private double speedLimit = 0.2;

    public RotatorSubsystem() {
        rotationMotorA = new WPI_TalonFX(RobotMap.Rotator.ROTATOR_MOTOR);
        rotationMotorA.setNeutralMode(NeutralMode.Brake);
        rotationMotorA.setInverted(true);
        rotationMotorB = new WPI_TalonFX(RobotMap.Rotator.ROTATOR_MOTOR_B);
        rotationMotorB.setNeutralMode(NeutralMode.Brake);
        rotationMotorB.setInverted(true);
        rotationMotorB.follow(rotationMotorA);

        pid = new PIDController(
                ROTATOR_P_COEFF,
                ROTATOR_I_COEFF,
                ROTATOR_D_COEFF);
        // pid.enableContinuousInput(0, Math.PI * 2);
        pid.setTolerance(Math.toRadians(30)); // TODO: TUNE THIS
        feedforward = new ArmFeedforward(
                ROTATOR_KS_COEFF,
                ROTATOR_KG_COEFF,
                ROTATOR_KV_COEFF,
                ROTATOR_KA_COEFF);

        encoder = new DutyCycleEncoder(0);
        // update the rotator PID to the current position
        setToCurrentPosition();
    }

    // Sets the setpoint to where the rotator is currently
    public void setToCurrentPosition() {
        setSetpoint(Math.toDegrees(getMeasurementRadians()));
    }

    // Returns the position of the rotator in radians
    private double getMeasurementRadians() {
        return Math.toRadians(getMeasurementDegrees());
    }

    // Returns the position of the rotator in degrees
    public double getMeasurementDegrees() {
        double currentPosition = encoder.getAbsolutePosition();
        double result = (((currentPosition * 360) + ENCODER_OFFSET) % 360);
        return result;
    }

    // Returns the setpoint of the rotator in degrees
    public double getSetpoint() {
        return Math.toDegrees(pid.getSetpoint());
    }

    // Sets the setpoint of the rotator degrees
    public void setSetpoint(double degrees) {
        pid.setSetpoint(Math.toRadians(degrees));
    }

    // Sets the speed of the rotator
    public void setSpeed(double speed) {
        rotationMotorA.set(speed); // Defaults to PercentOutput
    }

    // Returns true if the PID is at the setpoint, false otherwise
    public boolean atSetpoint() {
        return pid.atSetpoint();
    }

    @Override
    public void periodic() {
        // caluclate using the feedforward and PID
        double ff = feedforward.calculate(pid.getSetpoint() - (Math.PI / 2),
                rotationMotorA.getSelectedSensorVelocity());
        double speed = pid.calculate(getMeasurementRadians());
        // Constrain the calculation to the safe speed
        double safeSpeed = MathUtil.clamp(speed + ff, -speedLimit, speedLimit);
        // Set the speed of the rotator
        setSpeed(safeSpeed);

        if (log) {
            System.out.println("RotatorPIDOnly.periodic: current setpoint: " + Math.toDegrees(pid.getSetpoint()));
            System.out.println("RotatorPIDOnly.periodic: speed: " + speed);
            System.out.println("RotatorPIDOnly.periodic: ff: " + ff);
            System.out.println("RotatorPIDOnly.periodic: safe speed: " + safeSpeed);
        }
    }

}
