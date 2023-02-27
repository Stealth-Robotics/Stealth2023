package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class EndEffectorSubsystem extends SubsystemBase{
    private final Solenoid wristSolenoid;
    private final Solenoid chomperSolenoid;
    private final CANSparkMax motorA;
    public EndEffectorSubsystem() {
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

    public void toggleWrist(){
        wristSolenoid.toggle();
    }

    public void setWrist(boolean newValue){
        wristSolenoid.set(newValue);
    }

    public void toggleChomper(){
        chomperSolenoid.toggle();
    }

    public void setChomper(boolean newValue){
        chomperSolenoid.set(newValue);
    }

    public void close()
    {
        setWrist(false);
    }

    public void open()
    {
        setWrist(true);
    }

    public boolean getIsOpen(){
        return wristSolenoid.get();
    }
}
