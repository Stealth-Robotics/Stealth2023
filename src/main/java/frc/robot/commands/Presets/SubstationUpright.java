package frc.robot.commands.Presets;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AutoIntakeCommand;
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

public class SubstationUpright extends SequentialCommandGroup {
    public SubstationUpright(TelescopeSubsystem telescope, RotatorSubsystem rotator,
            CrocodileSubsystem crocodile, IntakeSubsystem intake, BooleanSupplier stopIntake,
            Supplier<Gamepiece> gamePiece) {
        addRequirements(telescope, rotator, crocodile);
        addCommands(
            new ParallelCommandGroup(
                new SequentialCommandGroup(
                    new RotatorToPosition(rotator, telescope, RotatorPosition.SHELF_UPRIGHT).withTimeout(2),
                    new TelescopeToPosition(telescope, TelescopePosition.SUBSTATION_UPRIGHT).withTimeout(2)
                    //new InstantCommand(() -> CommandScheduler.getInstance().schedule(new AutoIntakeCommand(intake, 1, stopIntake)))
//TODO: TUNE SPEED
                ),  
                new ConditionalCommand(
                    new InstantCommand(() -> crocodile.setWristSetpoint(WristPosition.SUBSTATION_UPRIGHT.getValue())),
                    crocodile.setWristToPositionCommand(WristPosition.CUBE_SCORE),
                    () -> (intake.getGamePiece() == Gamepiece.CONE)
                    
                ).withTimeout(2.5)
            ).until(stopIntake)
        );
    }
}
