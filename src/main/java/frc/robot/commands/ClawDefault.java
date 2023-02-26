package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClawSubsystem;

public class ClawDefault extends CommandBase {

    ClawSubsystem claw;

    public ClawDefault(ClawSubsystem claw) {
        this.claw = claw;        
    }
    

    @Override
    public void execute() {
        if(claw.getTrip() && claw.getIsOpen())
        {
            claw.close();
        }
    }
}
