package frc.robot.subsystems;

import javax.swing.text.Position;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class CrocodileSubsystem extends SubsystemBase {
    private final WPI_TalonFX intake;
    private final WPI_TalonFX wrist;
    private final PIDController pid;
    private final DutyCycleEncoder encoder;
    private final DigitalInput distanceSensor;

    public CrocodileSubsystem() {
        intake = new WPI_TalonFX(RobotMap.Crocodile.INTAKE);
        wrist = new WPI_TalonFX(RobotMap.Crocodile.WRIST);
        pid = new PIDController(0, 0, 0);
        encoder = new DutyCycleEncoder(1);
        distanceSensor = new DigitalInput(2);
        intake.setNeutralMode(NeutralMode.Brake);
        wrist.setNeutralMode(NeutralMode.Brake);
        /* enabled | Limit(amp) | Trigger Threshold(amp) | Trigger Threshold Time(s) */
        intake.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 35, 60, 0.1));
        setSetpoint(encoder.getAbsolutePosition());

    }

    public void setIntakeSpeed(double speed) {
        intake.set(speed);
    }

    private void setWristSpeed(double speed) {
        wrist.set(speed);
    }

    public void setSetpoint(double position) {
        pid.setSetpoint(position);
    }

    public double getSetpoint() {
        return pid.getSetpoint();
    }

    public double getAbsolutePosition() {
        return encoder.getAbsolutePosition();
    }

    public boolean getDistanceSensor() {
        return distanceSensor.get();
    }

    @Override
    public void periodic() {
        setWristSpeed(pid.calculate(encoder.getAbsolutePosition()));
    }
}
