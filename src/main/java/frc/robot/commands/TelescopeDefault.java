package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TelescopeSubsystem;

public class TelescopeDefault extends CommandBase {

    private TelescopeSubsystem telescopeSubsystem;
    private DoubleSupplier joystickSupplier;

    public TelescopeDefault(TelescopeSubsystem telescopeSubsystem, DoubleSupplier joystickSupplier) {
        this.telescopeSubsystem = telescopeSubsystem;
        this.joystickSupplier = joystickSupplier;
        addRequirements(telescopeSubsystem);

    }

    @Override
    public void initialize() {
        telescopeSubsystem.setSetpoint(telescopeSubsystem.getCurrentPosition());
    }

    @Override
    public void execute() {
        double joystickInput = joystickSupplier.getAsDouble();

        if (Math.abs(joystickInput) > 0.05) {
            if (telescopeSubsystem.inBounds()){
                telescopeSubsystem.setSpeed(MathUtil.clamp(joystickInput, -0.3, 0.3));
            }
        }
        else{
            telescopeSubsystem.setPosition(telescopeSubsystem.getCurrentPosition());
        }
    }
}
