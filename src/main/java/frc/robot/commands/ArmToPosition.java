package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Rotation;

public class ArmToPosition extends CommandBase{
    private final double position;
    private final Rotation arm;

    public ArmToPosition(Rotation arm, double position){
        this.position = position;
        this.arm = arm;
        addRequirements(arm);
    }

    @Override
    public void initialize(){
        arm.setSetpoint(position);
    }
    
    @Override
    public void execute(){
        arm.updatePosition();
    }
    
    @Override
    public boolean isFinished(){
        return arm.atSetpoint();
    }

    
}
