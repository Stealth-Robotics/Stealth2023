package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.RotatorSubsystem;
import frc.robot.subsystems.TelescopeSubsystem;
import frc.robot.subsystems.RotatorSubsystem.RotatorPosition;

public class RotatorToPosition extends CommandBase {
    private final RotatorSubsystem rotatorSubsystem;
    private final TelescopeSubsystem telescope;
    private final double setpoint;
    private final TelescopeToPosition resetTelescope;
    private boolean resettingElevator = false;

    public RotatorToPosition(RotatorSubsystem rotatorSubsystem, TelescopeSubsystem telescope, double setpoint) {
        this.rotatorSubsystem = rotatorSubsystem;
        this.setpoint = setpoint;
        this.telescope = telescope;
        addRequirements(rotatorSubsystem);

        resetTelescope = new TelescopeToPosition(telescope, 0);
    }

    public RotatorToPosition(RotatorSubsystem rotatorSubsystem, TelescopeSubsystem telescope, RotatorPosition position) {
        this.rotatorSubsystem = rotatorSubsystem;
        this.setpoint = position.getValue();
        this.telescope = telescope;
        addRequirements(rotatorSubsystem);

        resetTelescope = new TelescopeToPosition(telescope, 0);
    }

    // Set the setpoint to the desired position
    @Override
    public void initialize() {
        
        // // If the telescope is extended, and we are more than 2 telescope rotations away
        // // from the setpoint, reset the telescope.
        // if (telescope.getExtensionPercent() > 0.1/* 10 percent */) {
        //     // So we reset the telescope, and then come back to this command.
        //     CommandScheduler.getInstance()
        //             .schedule(resetTelescope.andThen(new RotatorToPosition(rotatorSubsystem, telescope, setpoint)));
        //     resettingElevator = true;
        // } else {
        //     rotatorSubsystem.setSetpoint(setpoint);
        // }
        rotatorSubsystem.setSetpoint(setpoint);
    }

    @Override
    public void execute() {
        rotatorSubsystem.setSetpoint(setpoint);
    }

    // If we are at the end, or we are resetting the elevator, end the command.
    @Override
    public boolean isFinished() {
        return rotatorSubsystem.atSetpoint() || resettingElevator;
    }

    // Keep the setpoint steady when we end
    @Override
    public void end(boolean interrupted) {
        rotatorSubsystem.setToCurrentPosition();
    }
}
