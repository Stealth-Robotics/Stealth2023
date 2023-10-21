package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class CrocodileSubsystem extends SubsystemBase {
    private final WPI_TalonFX wrist;
    private final PIDController wristPID;
    private final DutyCycleEncoder wristEncoder;
    private Gamepiece gamePiece = Gamepiece.CONE;
    private final double WRIST_kP = 0.01;
    private final double WRIST_kI = 0.0;
    private final double WRIST_kD = 0.00075;

    public final double SPEED_LIMIT = 0.75;

    private final double ENCODER_OFFSET = 42; //200;

    private boolean runPID = true;

    private final double LOWER_BOUND = 125;
    private final double UPPER_BOUND = 0;

    public enum WristPosition {
        CONE_PICKUP(123.3),
        CUBE_PICKUP(102),
        SUBSTATION_UPRIGHT(23),
        CONE_SCORE(42),
        CONE_HIGH(42),
        CUBE_SCORE(24.5),
        CONE_SHELF(137),
        CONE_STOW(-1.5),
        CUBE_SHELF(56.5);

        private final double value;

        private WristPosition(double position) {
            this.value = position;
        }

        public double getValue() {
            return value;
        }
    }

    public enum WristBoundState{
        IN_BOUNDS,
        OVER_UPPER_BOUND,
        UNDER_LOWER_BOUND
    }

    public CrocodileSubsystem() {
        wrist = new WPI_TalonFX(RobotMap.Crocodile.WRIST);
        wristPID = new PIDController(WRIST_kP, WRIST_kI, WRIST_kD);
        wristEncoder = new DutyCycleEncoder(RobotMap.Crocodile.WRIST_ENCODER_ID);;
        wrist.setNeutralMode(NeutralMode.Brake);
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
        double currentPosition = wristEncoder.getAbsolutePosition();
        double result = (((currentPosition * 360) - ENCODER_OFFSET) % 360);
        return result;
    }
    
    public Gamepiece getGamePiece() {
        return gamePiece;
    }

    public void setGamePiece(Gamepiece gamePiece) {
        this.gamePiece = gamePiece;
    }

    public void setRunPID(boolean set) {
        runPID = set;
    }

    public WristBoundState inBounds(){
        if(getWristPosition() > UPPER_BOUND){
            return WristBoundState.OVER_UPPER_BOUND;
        } else if(getWristPosition() < LOWER_BOUND){
            return WristBoundState.UNDER_LOWER_BOUND;
        } else {
            return WristBoundState.IN_BOUNDS;
        }
    }

    @Override
    public void periodic() {
        if (runPID) {
            double speed = MathUtil.clamp(wristPID.calculate(getWristPosition()), -SPEED_LIMIT, SPEED_LIMIT);
            setWristSpeed(speed);
        } 
        SmartDashboard.putString("Current Piece Selection", gamePiece.getData());
        System.out.println("wrist " + getWristPosition());
        System.out.println("setpoitn: " + getWristSetpoint());
        System.out.println(runPID);
        // System.out.println("wrist setpoint: " + getWristSetpoint());
        // System.out.println("speed: " + speed);
        // System.out.println(runPID);
        // System.out.println(getBeamBreak() + " " + getWristPosition());
    }
}
