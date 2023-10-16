package frc.robot.commands.DefaultCommands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.Gamepiece;

public class IntakeDefaultCommand extends CommandBase{
    IntakeSubsystem intake;
    DoubleSupplier trigger;

    public IntakeDefaultCommand(IntakeSubsystem intake, DoubleSupplier trigger){
        this.intake = intake;
        this.trigger = trigger;
        addRequirements(intake);
    }


    @Override
    public void execute() {
        double power = trigger.getAsDouble();
        //negate intake power if cube
        double multiplier = intake.getGamePiece() == Gamepiece.CONE ? 1 : -1;
        //something is in the intake
        if(!intake.getBeamBreak() && Math.abs(power) < 0.05){
            power += 0.15;
        }


        intake.setIntakeSpeed(power * multiplier);
    }
    
}
