package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.RotatorSubsystem;
import frc.robot.subsystems.TelescopeSubsystem;

public class RotatorToPosition extends CommandBase {
    private final RotatorSubsystem rotatorSubsystem;
    private final TelescopeSubsystem telescope;
    private final double setpoint;
    private final ResetTelescope resetTelescope;
    private boolean resettingElevator;
    public RotatorToPosition(RotatorSubsystem rotatorSubsystem, TelescopeSubsystem telescope, double setpoint)
    {
        this.rotatorSubsystem = rotatorSubsystem;
        this.setpoint = setpoint;
        this.telescope = telescope;
        addRequirements(rotatorSubsystem);

        resetTelescope = new ResetTelescope(telescope);
    }
    @Override
    public void initialize() {
        if (telescope.getCurrentPosition() > 2048 && Math.abs(telescope.getCurrentPosition() - setpoint) > 4096){
            CommandScheduler.getInstance().schedule(resetTelescope.andThen(new RotatorToPosition(rotatorSubsystem, telescope, setpoint)));
            resettingElevator = true;
        }
        else {
            rotatorSubsystem.setSetpoint(setpoint);
        }
    }
    @Override
    public boolean isFinished() {
        return rotatorSubsystem.getMeasurement() == setpoint || resettingElevator;
    }
    @Override
    public void end(boolean interrupted) {
        rotatorSubsystem.setSetpoint(rotatorSubsystem.getMeasurement());
    }
}
