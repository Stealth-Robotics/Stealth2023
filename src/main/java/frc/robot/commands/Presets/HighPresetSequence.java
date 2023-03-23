package frc.robot.commands.Presets;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.RotatorToPosition;
import frc.robot.commands.TelescopeToPosition;
import frc.robot.subsystems.CrocodileSubsystem;
import frc.robot.subsystems.RotatorSubsystem;
import frc.robot.subsystems.TelescopeSubsystem;
import frc.robot.subsystems.CrocodileSubsystem.WristPosition;
import frc.robot.subsystems.RotatorSubsystem.RotatorPosition;
import frc.robot.subsystems.TelescopeSubsystem.TelescopePosition;

public class HighPresetSequence extends SequentialCommandGroup {
    private DoubleSupplier intake;
    private double multiplier = 1; 
    public HighPresetSequence(TelescopeSubsystem telescope, RotatorSubsystem rotator, CrocodileSubsystem crocodile, DoubleSupplier intake, BooleanSupplier gamePiece) {
        this.intake = intake;
        addRequirements(rotator,telescope,crocodile);
        
        if(!gamePiece.getAsBoolean()){
            multiplier = -1;
        }
        addCommands(
            new InstantCommand(()->crocodile.setIntakeSpeed(0.25 * multiplier)),
            new RotatorToPosition(rotator, telescope, RotatorPosition.HIGH_SCORE).withTimeout(2),
            new TelescopeToPosition(telescope, TelescopePosition.HIGH_SCORE).withTimeout(2),
            crocodile.setWristToPositionCommand(WristPosition.CONE_SCORE).withTimeout(2)
            
        );
        deadlineWith(new InstantCommand(() -> crocodile.setIntakeSpeed(MathUtil.clamp((0.25 + intake.getAsDouble()), -1, 1) * multiplier)));
    }
}
