package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;

public class CrocodileSubsystem extends SubsystemBase {
    private final WPI_TalonFX intake;
    private final WPI_TalonFX wrist;
    private final PIDController wristPID;
    private final DutyCycleEncoder wristEncoder;
    private final DigitalInput beamBreak;
    private GamePiece gamePiece = GamePiece.CONE;
    private final double WRIST_kP = 0.01;
    private final double WRIST_kI = 0.0;
    private final double WRIST_kD = 0.00075;

    public final double SPEED_LIMIT = 0.75;

    private final double ENCODER_OFFSET = 0;

    private boolean runPID = true;

    private double offset = 0;

    public enum GamePiece {
        CONE("CONE"), 
        CUBE("CUBE");
        private final String name;

        private GamePiece(String name) {
            this.name = name;
        }
        String getData(){
            return name;
        }
    }

    public enum WristPosition {
        CONE_PICKUP(156),
        CUBE_PICKUP(113  ),
        CONE_SCORE(85),
        CUBE_SCORE(44),
        CONE_SHELF(56.5),
        CUBE_SHELF(56.5);

        private final double value;

        private WristPosition(double position) {
            this.value = position;
        }

        public double getValue() {
            return value;
        }
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
        wrist.setInverted(false);
        wristPID.setTolerance(360);
        // intake.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 35, 60, ENCODER_OFFSET));
        
    }

    public void onInit(){
        setToCurrentPosition();
    }

    public void setToCurrentPosition() {
        setWristSetpoint(getWristPosition());
    } 

    public void setIntakeSpeed(double speed) {
        intake.set(speed);
    }

    public void setWristSpeed(double speed) {
        wrist.set(speed);
    }

    public void setWristSetpoint(double position) {
        wristPID.setSetpoint(position);
    }

    public double getWristSetpoint() {
        return wristPID.getSetpoint();
    }

    private void setWristToPosition(WristPosition position) {
        setWristSetpoint(position.getValue());
    }

    public Command setWristToPositionCommand(WristPosition position) {
        return this.startEnd(
            () -> this.setWristToPosition(position), 
            () -> this.setToCurrentPosition())
            .until(() -> this.atSetpoint());
    }
    
    public boolean atSetpoint() {
        return wristPID.atSetpoint();
    }

    // In degrees
    public double getWristPosition() {
        double currentPosition = wristEncoder.getAbsolutePosition() + offset;
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

    public void setRunPID(boolean set) {
        runPID = set;
    }

    @Override
    public void periodic() {
        if (runPID) setWristSpeed(MathUtil.clamp(wristPID.calculate(getWristPosition()), -SPEED_LIMIT, SPEED_LIMIT));
        SmartDashboard.putBoolean("Beam Break Status", !getBeamBreak());
        SmartDashboard.putString("Current Piece Selection", gamePiece.getData());
        // System.out.println(getBeamBreak() + " " + getWristPosition());
    }
}
