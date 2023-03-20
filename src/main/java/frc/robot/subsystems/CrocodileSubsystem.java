package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class CrocodileSubsystem extends SubsystemBase {
    private final WPI_TalonFX intake;
    private final WPI_TalonFX wrist;
    private final PIDController wristPID;
    private final DutyCycleEncoder wristEncoder;
    private final DigitalInput beamBreak;
    private GamePiece gamePiece = GamePiece.CONE;
    //TODO: Tune PID
    private final double WRIST_kP = 1.0;
    private final double WRIST_kI = 0.0;
    private final double WRIST_kD = 0.0;

    // TODO: Set speed limit
    private final double SPEED_LIMIT = 0.0;

    // Offset of the encoder. See diagram above for reference
    private final double ENCODER_OFFSET = 0.3;

    public enum GamePiece {
        CONE, 
        CUBE
    }

    public CrocodileSubsystem() {
        intake = new WPI_TalonFX(RobotMap.Crocodile.INTAKE);
        wrist = new WPI_TalonFX(RobotMap.Crocodile.WRIST);
        wristPID = new PIDController(WRIST_kP, WRIST_kI, WRIST_kD);
        wristEncoder = new DutyCycleEncoder(RobotMap.Crocodile.WRIST_ENCODER_ID);
        beamBreak = new DigitalInput(RobotMap.Crocodile.BEAM_BREAK_ID);
        intake.setNeutralMode(NeutralMode.Brake);
        wrist.setNeutralMode(NeutralMode.Brake);
        intake.setInverted(true);
    }

    public void setToCurrentPosition() {
        setWristSetpoint(wristEncoder.getAbsolutePosition());
    }

    public void setIntakeSpeed(double speed) {
        intake.set(speed);
    }

    private void setWristSpeed(double speed) {
        wrist.set(speed);
    }

    public void setWristSetpoint(double position) {
        wristPID.setSetpoint(position);
    }

    public double getWristSetpoint() {
        return wristPID.getSetpoint();
    }

    public boolean atSetpoint() {
        return wristPID.atSetpoint();
    }

    // In degrees
    public double getWristPosition() {
        double currentPosition = wristEncoder.getAbsolutePosition();
        double result = (((currentPosition * 360) - ENCODER_OFFSET) % 360);
        return result;
    }

    public boolean getBeamBreak() {
        return beamBreak.get();
    }
    
    public GamePiece getGamePiece() {
        return gamePiece;
    }

    public void setGamePiece(GamePiece gamePiece) {
        this.gamePiece = gamePiece;
    }

    @Override
    public void periodic() {
        setWristSpeed(MathUtil.clamp(wristPID.calculate(wristEncoder.getAbsolutePosition()), -SPEED_LIMIT, SPEED_LIMIT));
    }
}
