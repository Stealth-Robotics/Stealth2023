package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.RotatorSubsystem;
import frc.robot.subsystems.TelescopeSubsystem;

public class RotatorToPosition extends CommandBase {
    private final RotatorSubsystem rotatorSubsystem;
    private final TelescopeSubsystem telescope;
    private final double setpoint;
    private final TelescopeToPosition resetTelescope;
    private boolean resettingElevator;

    public RotatorToPosition(RotatorSubsystem rotatorSubsystem, TelescopeSubsystem telescope, double setpoint) {
        this.rotatorSubsystem = rotatorSubsystem;
        this.setpoint = setpoint;
        this.telescope = telescope;
        addRequirements(rotatorSubsystem);

        resetTelescope = new TelescopeToPosition(telescope, 0);
    }

    @Override
    public void initialize() {
        // 4096 ticks is two rotations. I'm not going to change this atm because i dont
        // care about percent right now, I care about actual rotations. This can be
        // addressed after GPK
        // if (telescope.currentTicksToPercent() > 0.1/* 10 percent */ && Math.abs(telescope.getCurrentPosition() - setpoint) > 4096) {
        //     CommandScheduler.getInstance()
        //             .schedule(resetTelescope.andThen(new RotatorToPosition(rotatorSubsystem, telescope, setpoint)));
        //     resettingElevator = true;
        // } else {
            rotatorSubsystem.setSetpoint(setpoint);
        // }
    }

    @Override
    public boolean isFinished() {
        return rotatorSubsystem.atSetpoint() || resettingElevator;
    }

    @Override
    public void end(boolean interrupted) {
        rotatorSubsystem.setToCurrentPosition();
    }
}
