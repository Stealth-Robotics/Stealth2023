package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
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
    private final WPI_TalonFX rotationMotor;
    private final PIDController pid;
    private final DutyCycleEncoder encoder;

    
    private boolean log = false;

    
    private double speedLimit = 1; 
    private final ArmFeedforward feedforward;

    public RotatorSubsystem() {

        rotationMotor = new WPI_TalonFX(RobotMap.Rotator.ROTATOR_MOTOR);
        rotationMotor.setNeutralMode(NeutralMode.Coast);
        rotationMotor.setInverted(true);

        pid = new PIDController(
                Constants.RotatorConstants.ROTATOR_P_COEFF,
                Constants.RotatorConstants.ROTATOR_I_COEFF,
                Constants.RotatorConstants.ROTATOR_D_COEFF);
        // pid.enableContinuousInput(0, Math.PI * 2);
        pid.setTolerance(Math.toRadians(3));
        feedforward = new ArmFeedforward(
                Constants.RotatorConstants.ROTATOR_KS_COEFF,
                Constants.RotatorConstants.ROTATOR_KG_COEFF,
                Constants.RotatorConstants.ROTATOR_KV_COEFF,
                Constants.RotatorConstants.ROTATOR_KA_COEFF);

        encoder = new DutyCycleEncoder(0);
        reset();
    }

    public void reset() {
        setSetpoint(Math.toDegrees(getMeasurement()));
    }

    private double getMeasurement() {
        double currentPosition = encoder.getAbsolutePosition();
        double result = Math.toRadians(((currentPosition * 360) + Constants.RotatorConstants.ENCODER_OFFSET) % 360);
        if (log) {
            System.out.println("RotatorPIDOnly.getMeasurement: Current encoder position (raw): " + currentPosition);
            System.out.println(
                    "RotatorPIDOnly.getMeasurement: Current encoder position (adj): " + Math.toDegrees(result));
        }
        SmartDashboard.putNumber("RotatorPIDOnly.getMeasurement: Current encoder position (raw): ", currentPosition);
        SmartDashboard.putNumber("RotatorPIDOnly.getMeasurement: Current encoder position (adj): ", Math.toDegrees(result));
        return result;
    }

    public double getSetpoint() {
        return Math.toDegrees(pid.getSetpoint());
    }

    public void setSetpoint(double degrees) {
        pid.setSetpoint(Math.toRadians(degrees));
    }

    //The SetPOINT value is in degrees
    public void setGoal(double setPoint) {
        setSetpoint(setPoint);
    }

    private void setSpeed(double speed) {
        rotationMotor.set(speed); // Defaults to PercentOutput
    }

    @Override
    public void periodic() {
        double ff = feedforward.calculate(pid.getSetpoint() - (Math.PI / 2), rotationMotor.getSelectedSensorVelocity());
        double speed = pid.calculate(getMeasurement());
        double safeSpeed = MathUtil.clamp(speed + ff, -speedLimit, speedLimit);
        if (log) {
            System.out.println("RotatorPIDOnly.periodic: current setpoint: " + Math.toDegrees(pid.getSetpoint()));
            System.out.println("RotatorPIDOnly.periodic: speed: " + speed);
            System.out.println("RotatorPIDOnly.periodic: ff: " + ff);
            System.out.println("RotatorPIDOnly.periodic: safe speed: " + safeSpeed);
        }
        SmartDashboard.putNumber("Setpoint: " , Math.toDegrees(pid.getSetpoint()));
        SmartDashboard.putNumber("Speed: " , speed);        
        SmartDashboard.putNumber("Setpoint: " , ff);
        SmartDashboard.putNumber("safe speed: " , safeSpeed);
        SmartDashboard.putNumber("Current Pos:", Math.toDegrees(getMeasurement()));

        


        setSpeed(speed+ff);
    }

}
