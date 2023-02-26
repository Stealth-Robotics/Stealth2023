package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotMap;

public class ClawSubsystem extends SubsystemBase{
    private Solenoid clawSolenoid;

    public ClawSubsystem() {

        clawSolenoid = new Solenoid(
        RobotMap.Pneumatics.PCM, 
        RobotMap.Pneumatics.PCM_TYPE, 
        RobotMap.Pneumatics.CLAW_PCM_CHANNEL);
    }

    public void toggle(){
        clawSolenoid.toggle();
    }

    public void set(boolean newValue){
        clawSolenoid.set(newValue);
    }

    public void close()
    {
        set(false);
    }

    public void open()
    {
        set(true);
    }

    public boolean getIsOpen(){
        return clawSolenoid.get();
    }
}
