package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Intake;

public class IntakeOutUpdateSetpoint extends SequentialCommandGroup {
    public IntakeOutUpdateSetpoint(Intake intake, PIDController pid, double setpoint) {
        addCommands(
            //command is scheduled when setpoint will kill intake, so we first extend intake so it doesn't die
                new InstantCommand(() -> intake.deployIntake()),
                //after we extend the intake, wait half a secondd so that we make sure we don't kill it
                new WaitCommand(0.5),
                new InstantCommand(() -> pid.setSetpoint(setpoint))
                );
    }

}
