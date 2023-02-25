package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotMap;

public class RotatorPIDOnly extends SubsystemBase {
    private final WPI_TalonFX rotationMotor;
    private final PIDController pid;
    private final DutyCycleEncoder encoder;

    public RotatorPIDOnly() {
        rotationMotor = new WPI_TalonFX(RobotMap.Rotator.ROTATOR_MOTOR);
        pid = new PIDController(
            Constants.RotatorConstants.ROTATOR_P_COEFF, 
            Constants.RotatorConstants.ROTATOR_I_COEFF, 
            Constants.RotatorConstants.ROTATOR_D_COEFF);
        pid.enableContinuousInput(0, Math.PI * 2);
        this.encoder = new DutyCycleEncoder(0);
        setSetpoint(getMeasurement());
    }

    private double getMeasurement() {
        return Math.toRadians(((encoder.getAbsolutePosition() * 360) + Constants.RotatorConstants.ENCODER_OFFSET)%360);
    }

    public double getSetpoint() {
        return pid.getSetpoint();
    }

    public void setSetpoint(double setPoint) {
        pid.setSetpoint(setPoint);
    }

    private void setSpeed(double speed) {
        rotationMotor.set(speed);
    }

    @Override
    public void periodic() {
        setSpeed(pid.calculate(getMeasurement()));
    }
    
}
