package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
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
import frc.robot.commands.ResetTelescope;

public class RotatorSubsystem extends SubsystemBase {
    private final WPI_TalonFX rotationMotorA;
    private final WPI_TalonFX rotationMotorB;
    private final PIDController pid;
    private final DutyCycleEncoder encoder;
    
    private boolean log = false;

    
    private double speedLimit = 0.2; 
    private final ArmFeedforward feedforward;

    public RotatorSubsystem() {
        rotationMotorA = new WPI_TalonFX(RobotMap.Rotator.ROTATOR_MOTOR);
        rotationMotorA.setNeutralMode(NeutralMode.Brake);
        rotationMotorA.setInverted(true);
        rotationMotorB = new WPI_TalonFX(-1);
        rotationMotorB.setNeutralMode(NeutralMode.Brake);
        rotationMotorB.setInverted(true);
        rotationMotorB.follow(rotationMotorA); 

        pid = new PIDController(
                Constants.RotatorConstants.ROTATOR_P_COEFF,
                Constants.RotatorConstants.ROTATOR_I_COEFF,
                Constants.RotatorConstants.ROTATOR_D_COEFF);
        // pid.enableContinuousInput(0, Math.PI * 2);
        pid.setTolerance(Math.toRadians(30));
        feedforward = new ArmFeedforward(
                Constants.RotatorConstants.ROTATOR_KS_COEFF,
                Constants.RotatorConstants.ROTATOR_KG_COEFF,
                Constants.RotatorConstants.ROTATOR_KV_COEFF,
                Constants.RotatorConstants.ROTATOR_KA_COEFF);

        encoder = new DutyCycleEncoder(0);
        setToCurrentPosition();
    }

    public void setToCurrentPosition() {
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
    
        return result;
    }

    public double getMeasurementDegrees() {
        double currentPosition = encoder.getAbsolutePosition();
        double result = (((currentPosition * 360) + Constants.RotatorConstants.ENCODER_OFFSET) % 360);
    
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

    public void setSpeed(double speed) {
        rotationMotorA.set(speed); // Defaults to PercentOutput
    }

    public boolean atSetpoint(){
        return pid.atSetpoint();
    }

    @Override
    public void periodic() {
        double ff = feedforward.calculate(pid.getSetpoint() - (Math.PI / 2), rotationMotorA.getSelectedSensorVelocity());
        double speed = pid.calculate(getMeasurement());
        double safeSpeed = MathUtil.clamp(speed + ff, -speedLimit, speedLimit);
        if (log) {
            System.out.println("RotatorPIDOnly.periodic: current setpoint: " + Math.toDegrees(pid.getSetpoint()));
            System.out.println("RotatorPIDOnly.periodic: speed: " + speed);
            System.out.println("RotatorPIDOnly.periodic: ff: " + ff);
            System.out.println("RotatorPIDOnly.periodic: safe speed: " + safeSpeed);
        }        
        setSpeed(safeSpeed);
        // System.out.println("Rotator position: " + getMeasurementDegrees() + " | SP: " + getSetpoint());
    }

}
