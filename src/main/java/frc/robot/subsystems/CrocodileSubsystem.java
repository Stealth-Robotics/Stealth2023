package frc.robot.subsystems;

import javax.swing.text.Position;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class CrocodileSubsystem extends SubsystemBase {
    private final WPI_TalonFX intake;
    private final WPI_TalonFX wrist;
    private final PIDController wristPID;
    private final DutyCycleEncoder encoder;
    private final DigitalInput beamBreak;

    //TODO: Set speed limit
    private final double SPEED_LIMIT = 0;

    public CrocodileSubsystem() {
        intake = new WPI_TalonFX(RobotMap.Crocodile.INTAKE);
        wrist = new WPI_TalonFX(RobotMap.Crocodile.WRIST);
        //TODO: Tune PID
        wristPID = new PIDController(0, 0, 0);
        encoder = new DutyCycleEncoder(RobotMap.Crocodile.WRIST_ENCODER_ID);
        beamBreak = new DigitalInput(RobotMap.Crocodile.BEAM_BREAK_ID);
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
        wristPID.setSetpoint(position);
    }

    public double getSetpoint() {
        return wristPID.getSetpoint();
    }

    public double getAbsolutePosition() {
        return encoder.getAbsolutePosition();
    }

    public boolean getBeamBreak() {
        return beamBreak.get();
    }

    @Override
    public void periodic() {
        setWristSpeed(MathUtil.clamp(wristPID.calculate(encoder.getAbsolutePosition()), -SPEED_LIMIT, SPEED_LIMIT));
    }
}
