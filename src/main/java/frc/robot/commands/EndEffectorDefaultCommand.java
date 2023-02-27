package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.EndEffectorSubsystem;

public class EndEffectorDefaultCommand extends CommandBase {
    private final EndEffectorSubsystem subsystem;
    private final BooleanSupplier toggleWirst;
    private final DoubleSupplier motorSpeed;
    private final BooleanSupplier toggleChomper;
    public EndEffectorDefaultCommand(EndEffectorSubsystem subsystem, BooleanSupplier toggleChomper, BooleanSupplier toggleWirst, DoubleSupplier motorSpeed) {
        this.subsystem = subsystem;
        this.toggleChomper = toggleChomper;
        this.toggleWirst = toggleWirst;
        this.motorSpeed = motorSpeed;
        addRequirements(subsystem);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (toggleWirst.getAsBoolean()){
            subsystem.toggleWrist();
        }
        if (toggleChomper.getAsBoolean()){
            subsystem.toggleChomper();
        }
        subsystem.setMotorSpeed(motorSpeed.getAsDouble());
    }    
}
