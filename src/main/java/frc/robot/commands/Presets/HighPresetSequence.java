package frc.robot.commands.Presets;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.RotatorToPosition;
import frc.robot.commands.TelescopeToPosition;
import frc.robot.subsystems.CrocodileSubsystem;
import frc.robot.subsystems.CrocodileSubsystem.WristPosition;
import frc.robot.subsystems.Gamepiece;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.RotatorSubsystem;
import frc.robot.subsystems.RotatorSubsystem.RotatorPosition;
import frc.robot.subsystems.Swerve.DrivebaseSubsystem;
import frc.robot.subsystems.TelescopeSubsystem;
import frc.robot.subsystems.TelescopeSubsystem.TelescopePosition;

public class HighPresetSequence extends SequentialCommandGroup {
    private Command runIntake;
    public HighPresetSequence(TelescopeSubsystem telescope, RotatorSubsystem rotator, CrocodileSubsystem crocodile,
            IntakeSubsystem intakeSubsystem, DoubleSupplier intake, Supplier<Gamepiece> piece) 
    {
        addRequirements(rotator, telescope, crocodile);
        //thank you @mikemag for this
        DoubleSupplier multiplier = () -> piece.get() == Gamepiece.CONE ? 1 : -1;
        
        if (intake != null){
            runIntake = new RunCommand(() -> intakeSubsystem.setIntakeSpeed(
                MathUtil.clamp((0.25 + intake.getAsDouble()), -1, 1) * multiplier.getAsDouble()));
        }
        else {
            runIntake = new InstantCommand();
        }
        Command teleRotator = new ConditionalCommand(
            new SequentialCommandGroup(
                new RotatorToPosition(rotator, telescope, RotatorPosition.HIGH_SCORE).withTimeout(2),
                new TelescopeToPosition(telescope, TelescopePosition.HIGH_SCORE).withTimeout(2)
            ),
            new SequentialCommandGroup(
                new RotatorToPosition(rotator, telescope, RotatorPosition.CUBE_HIGH).withTimeout(2),
                new TelescopeToPosition(telescope, TelescopePosition.CUBE_HIGH).withTimeout(2)
            ),
            () -> intakeSubsystem.getGamePiece() == Gamepiece.CONE
        );
        
        addCommands(
            new SequentialCommandGroup(
                new InstantCommand(() -> intakeSubsystem.setIntakeSpeed(0.25 * multiplier.getAsDouble())),
                new ParallelCommandGroup(
                    teleRotator,
                    new ConditionalCommand(
                        new InstantCommand(() -> crocodile.setWristSetpoint(WristPosition.CONE_HIGH.getValue())),
                        new InstantCommand(() -> crocodile.setWristSetpoint(WristPosition.CUBE_SCORE.getValue())), 
                        () -> intakeSubsystem.getGamePiece() == Gamepiece.CONE).withTimeout(1.5)
                )
            ).deadlineWith(runIntake)
        );
        
    }
}
