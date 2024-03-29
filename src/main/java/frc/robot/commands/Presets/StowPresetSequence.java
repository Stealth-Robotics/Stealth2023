package frc.robot.commands.Presets;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AutoIntakeCommand;
import frc.robot.commands.RotatorToPosition;
import frc.robot.commands.TelescopeToPosition;
import frc.robot.subsystems.CrocodileSubsystem;
import frc.robot.subsystems.RotatorSubsystem;
import frc.robot.subsystems.TelescopeSubsystem;
import frc.robot.subsystems.CrocodileSubsystem.GamePiece;
import frc.robot.subsystems.CrocodileSubsystem.WristPosition;
import frc.robot.subsystems.TelescopeSubsystem.TelescopePosition;

public class StowPresetSequence extends SequentialCommandGroup {
    Command runIntake;
    public StowPresetSequence(TelescopeSubsystem telescope, RotatorSubsystem rotator, CrocodileSubsystem crocodile, DoubleSupplier intake, Supplier<GamePiece> gamePiece) {
        addRequirements(telescope, rotator, crocodile);

        //Thank you @mikemag for this
        DoubleSupplier multiplier = () -> gamePiece.get() == GamePiece.CONE ? 1 : -1;
        if (intake != null){
            runIntake = new RunCommand(() -> crocodile.setIntakeSpeed(
                MathUtil.clamp((0.25 + intake.getAsDouble()), -1, 1) * multiplier.getAsDouble()));
        }
        else {
            runIntake = new InstantCommand();
        }

        addCommands(
            new SequentialCommandGroup(
                new InstantCommand(() -> crocodile.setIntakeSpeed(0.25 * multiplier.getAsDouble())),
                new TelescopeToPosition(telescope, TelescopePosition.RETRACTED).withTimeout(2),
                new RotatorToPosition(rotator, telescope, 90).withTimeout(2),
                crocodile.setWristToPositionCommand(WristPosition.CONE_SCORE).withTimeout(2)
            ).withTimeout(2.5).deadlineWith(runIntake)
        );
    }
}
