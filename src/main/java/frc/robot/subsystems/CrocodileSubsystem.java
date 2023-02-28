package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class CrocodileSubsystem extends SubsystemBase{
    private final Solenoid wristSolenoid;
    private final Solenoid chomperSolenoid;
    private final CANSparkMax motorA;

    private double currentMotorPower;
    public CrocodileSubsystem() {
        motorA = new CANSparkMax(RobotMap.EndEffector.END_EFFECTOR_MOTOR_A, MotorType.kBrushless);
        wristSolenoid = new Solenoid(
            RobotMap.Pneumatics.PCM, 
            RobotMap.Pneumatics.PCM_TYPE, 
            RobotMap.Pneumatics.CLAW_PCM_CHANNEL);
        chomperSolenoid = new Solenoid(
            RobotMap.Pneumatics.PCM, 
            RobotMap.Pneumatics.PCM_TYPE, 
            RobotMap.Pneumatics.CHOMPER_PCM_CHANNEL);
    }

    public void setMotorSpeed(double speed){
        motorA.set(speed);
    }


    private void setWrist(boolean newValue){
        wristSolenoid.set(newValue);
    }

    public void toggleWrist(){
        wristSolenoid.toggle();
    }
    
    public boolean getWristPositionBool(){
        return wristSolenoid.get();
    }

    public void wristUp(){
        setWrist(true);
    }

    public void wristDown(){
        setWrist(false);
    }
        

    private void setChomper(boolean newValue){
        chomperSolenoid.set(newValue);
    }

    public void toggleChomper(){
        chomperSolenoid.toggle();
    }

    public void closeChomper()
    {
        setChomper(true);
    }

    public void openChomper()
    {
        setChomper(false);
    }

    public boolean getChomperPositionBool(){
        return chomperSolenoid.get();
    }

    @Override
    public void periodic() {
        
    }
}
