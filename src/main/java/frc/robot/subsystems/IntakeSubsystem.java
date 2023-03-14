package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase{
    private final WPI_TalonFX intakeMotor;
    private final WPI_TalonFX wristMotor;
    private final DigitalInput beamBreak;
    private final DutyCycleEncoder encoder;
    private final PIDController wristPid;
    //assign ports
    private final int BEAM_BREAK_PORT = 0;
    private final int ENCODER_PORT = 0;
    private final int INTAKE_PORT = 0;
    private final int WRIST_PORT = 0;
    //variables for lower and upper wrist limit
    //TODO: get wrist limits
    private final double WRIST_LOWER_LIMIT = 0;
    private final double WRIST_UPPER_LIMIT = 0;
    
    
    public IntakeSubsystem(){
        intakeMotor = new WPI_TalonFX(INTAKE_PORT);
        wristMotor = new WPI_TalonFX(WRIST_PORT);
        beamBreak = new DigitalInput(BEAM_BREAK_PORT);
        encoder = new DutyCycleEncoder(ENCODER_PORT);
        wristPid = new PIDController(0, 0, 0);
    }
    //set intake speed method
    public void setIntakeSpeed(double speed){
        intakeMotor.set(ControlMode.PercentOutput, speed);
    }
    public double getWristPosition(){
        return encoder.getAbsolutePosition();
    }
    public boolean getBeamBreak(){
        return beamBreak.get();
    }
    public void setWristSetpoint(double position){
        wristPid.setSetpoint(position);
    }
    public double getWristsSetpoint(){
        return wristPid.getSetpoint();
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        //set wrist position to pid clamping between upper and lower limits
        wristMotor.set(ControlMode.PercentOutput, MathUtil.clamp(wristPid.calculate(getWristPosition()), WRIST_LOWER_LIMIT, WRIST_UPPER_LIMIT));
        
    }

    
}
