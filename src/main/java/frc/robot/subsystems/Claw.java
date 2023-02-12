package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class Claw extends SubsystemBase{
    private Solenoid clawSolenoid;

    public Claw(){
        clawSolenoid = new Solenoid(
        RobotMap.Pneumatics.PCM, 
        RobotMap.Pneumatics.PCM_TYPE, 
        RobotMap.Pneumatics.CLAW_PCM_CHANNEL);
    }

    public void toggleClaw(){
        clawSolenoid.toggle();
    }

    public boolean getIsOpen(){
        return clawSolenoid.get();
    }
}
