package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Intake;

public class IntakeOutUpdateSetpoint extends SequentialCommandGroup {
    public IntakeOutUpdateSetpoint(Intake intake, PIDController pid, double setpoint) {
        addCommands(
                new InstantCommand(() -> intake.deployIntake()),
                new WaitCommand(0.5),
                new InstantCommand(() -> pid.setSetpoint(setpoint))
                );
    }

}
