package frc.robot.commands.Presets;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.RotatorToPosition;
import frc.robot.commands.TelescopeToPosition;
import frc.robot.subsystems.CrocodileSubsystem;
import frc.robot.subsystems.RotatorSubsystem;
import frc.robot.subsystems.TelescopeSubsystem;
import frc.robot.subsystems.Gamepiece;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.CrocodileSubsystem.WristPosition;
import frc.robot.subsystems.RotatorSubsystem.RotatorPosition;
import frc.robot.subsystems.TelescopeSubsystem.TelescopePosition;

public class StowPresetSequence extends SequentialCommandGroup {
    Command runIntake;
    public StowPresetSequence(TelescopeSubsystem telescope, RotatorSubsystem rotator, CrocodileSubsystem crocodile, IntakeSubsystem intakeSubsystem, DoubleSupplier intake,  Supplier<Gamepiece> gamePiece) {
        addRequirements(telescope, rotator, crocodile);

        //Thank you @mikemag for this
        DoubleSupplier multiplier = () -> gamePiece.get() == Gamepiece.CONE ? 1 : -1;
        if (intake != null){
            runIntake = new RunCommand(() -> intakeSubsystem.setIntakeSpeed(
                MathUtil.clamp((0.25 + intake.getAsDouble()), -1, 1) * multiplier.getAsDouble()));
        }
        else {
            runIntake = new InstantCommand();
        }

        addCommands(
            new ParallelCommandGroup(
                new SequentialCommandGroup(
                    new InstantCommand(() -> intakeSubsystem.setIntakeSpeed(0.25 * multiplier.getAsDouble())),
                    new TelescopeToPosition(telescope, TelescopePosition.RETRACTED).withTimeout(2),
                    new RotatorToPosition(rotator, telescope, RotatorPosition.STOW.getValue()).withTimeout(2)
                ).withTimeout(3.0).deadlineWith(runIntake),
                new SequentialCommandGroup(
                    new WaitCommand(0.25),
                    new InstantCommand(() -> crocodile.setWristSetpoint(WristPosition.CONE_STOW.getValue()))
                )
            )
        );
    }
}
