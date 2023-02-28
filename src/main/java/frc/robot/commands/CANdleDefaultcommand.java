package frc.robot.commands;

import java.util.function.BooleanSupplier;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotMap.CANdle;
import frc.robot.subsystems.CANdleSubsystem;

public class CANdleDefaultcommand extends CommandBase{
    private final CANdleSubsystem candle;
    private final BooleanSupplier purpleButton, yellowButton, chomp;
    public CANdleDefaultcommand(CANdleSubsystem candle, BooleanSupplier purpleButton, BooleanSupplier yellowButton,
            BooleanSupplier chomp) {
        this.candle = candle;
        this.purpleButton = purpleButton;
        this.yellowButton = yellowButton;
        this.chomp = chomp;
        addRequirements(candle);
    }
    @Override
    public void execute(){
        if(chomp.getAsBoolean()) candle.idle();
        if(purpleButton.getAsBoolean()) candle.purple();
        if(yellowButton.getAsBoolean()) candle.yellow();
    }

    
    
    
}
