package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.Command;
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
    private static final double ROTATOR_P_COEFF = 1;
    private static final double ROTATOR_I_COEFF = 0.0005;
    private static final double ROTATOR_D_COEFF = 0.075;
    // Feedforward Constants
    private static final double ROTATOR_KS_COEFF = 0;
    private double ROTATOR_KG_COEFF_RETRACTED = 0.065;
    private double ROTATOR_KG_COEFF_EXTENDED = 0.07;
    // Volt Second Per Rad
    private static final double ROTATOR_KV_COEFF = 0;
    // Volt Second Squared Per Rad
    private static final double ROTATOR_KA_COEFF = 0;
    // Offset of the encoder. See diagram above for reference
    private static final double ENCODER_OFFSET = -157 + 180;// (157.36 - 180);//(0.18439 * 360); //138;
    // Bounds of the rotator, degrees
    private static final double HIGH_BOUND = 285;
    private static final double LOW_BOUND = 70;

    private final PIDController pid;
    // Absolute encoder
    private final DutyCycleEncoder encoder;
    // Wether or not to log data in periodic
    private boolean log = false;
    // The speed limit of the rotator
    // This is for safety of people and robot
    private double speedLimit = 1;

    private boolean runPID = true;

    //TODO: set to actual position values
    public enum RotatorPosition {
        GROUND_PICKUP(-1), 
        SHELF_PICKUP(-1), 
        HIGH_SCORE(-1), 
        MID_SCORE(-1), 
        LOW_SCORE(-1);

        private final int value;
        private RotatorPosition(int position){
            this.value = position;
        }
        public int getValue() {
            return value;
        }
    }

    private TelescopeSubsystem telescope;

    private double currentKG = ROTATOR_KG_COEFF_RETRACTED;

    public RotatorSubsystem(TelescopeSubsystem telescope) {
        rotationMotorA = new WPI_TalonFX(RobotMap.Rotator.ROTATOR_MOTOR);
        rotationMotorA.setNeutralMode(NeutralMode.Brake);
        rotationMotorB = new WPI_TalonFX(RobotMap.Rotator.ROTATOR_MOTOR_B);
        rotationMotorB.setNeutralMode(NeutralMode.Brake);
        rotationMotorB.follow(rotationMotorA);

        pid = new PIDController(
                ROTATOR_P_COEFF,
                ROTATOR_I_COEFF,
                ROTATOR_D_COEFF);

        // pid.setTolerance(Math.toRadians(6));

        encoder = new DutyCycleEncoder(0);

        this.telescope = telescope;
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

    public void togglePID() {
        runPID = !runPID;
    }

    public void setRunPID(boolean set) {
        runPID = set;
    }

    public void setKg(double F) {
        currentKG = F;
    }

    public double getKg(){
        return currentKG;
    }

    public double getRetractedKg(){
        return ROTATOR_KG_COEFF_RETRACTED;
    }

    public double getExtendedKg(){
        return ROTATOR_KG_COEFF_EXTENDED;
    }

    public void setRetractedKg(double kg){
        ROTATOR_KG_COEFF_RETRACTED = kg;
    }

    public void setExtendedKg(double kg){
        ROTATOR_KG_COEFF_EXTENDED = kg;
    }

    public double calculateF(double extensionPercent) {
        return ((ROTATOR_KG_COEFF_EXTENDED - ROTATOR_KG_COEFF_RETRACTED) * extensionPercent)
                + ROTATOR_KG_COEFF_RETRACTED;
    }

    // Calculation from WPILib ArmFeedForward
    // https://docs.wpilib.org/en/stable/docs/software/advanced-controls/introduction/introduction-to-feedforward.html
    // Implementation from their class:
    // https://docs.wpilib.org/en/stable/docs/software/advanced-controls/controllers/feedforward.html#armfeedforward
    // Copyright (c) FIRST and other WPILib contributors.
    public double calculateFeedForward(
    
        double positionRadians, double velocityRadPerSec, double accelRadPerSecSquared) {
        return ROTATOR_KS_COEFF * Math.signum(velocityRadPerSec)
            + currentKG * Math.sin(positionRadians)
            + ROTATOR_KV_COEFF * velocityRadPerSec
            + ROTATOR_KA_COEFF * accelRadPerSecSquared;
    }

    @Override
    public void periodic() {
        setKg(calculateF(telescope.getExtensionPercent()));

        // caluclate using the feedforward and PID
        double ff = -calculateFeedForward(getMeasurementRadians() -  Math.PI,
                rotationMotorA.getSelectedSensorVelocity(), 0);
        double speed = pid.calculate(getMeasurementRadians());
        // Constrain the calculation to the safe speed
        double safeSpeed = MathUtil.clamp(speed + ff, -speedLimit, speedLimit);
        // Set the speed of the rotator
        if (runPID) setSpeed(safeSpeed);
        // System.out.println("Result " + ff);
        // System.out.println( safeSpeed + " | M:" + getMeasurementDegrees() + " | SP:" + Math.toDegrees(pid.getSetpoint()));
        if (log) {
            System.out.println("RotatorPIDOnly.periodic: current setpoint: " + Math.toDegrees(pid.getSetpoint()));
            System.out.println("RotatorPIDOnly.periodic: speed: " + speed);
            System.out.println("RotatorPIDOnly.periodic: ff: " + ff);
            System.out.println("RotatorPIDOnly.periodic: safe speed: " + safeSpeed);
        }
        System.out.println(runPID + " rotator " + getMeasurementDegrees() + " SP " + pid.getSetpoint()+ " pwr "+ speed);
    }

    @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    builder.setSmartDashboardType("FeedforwardTuning");
    builder.addDoubleProperty("current Kg", this::getKg, this::setKg);
    builder.addDoubleProperty("retracted Kg", this::getRetractedKg, this::setRetractedKg);
    builder.addDoubleProperty("extended Kg", this::getExtendedKg, this::setExtendedKg);

  }

}
