package frc.robot.commands.DefaultCommands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.CrocodileSubsystem;

public class CrocodileDefaultCommand extends CommandBase {
    private final CrocodileSubsystem subsystem;
    //One of the joystick axes of the controller
    private final DoubleSupplier motorSpeed;
    //The button on the controller that slows the crocodile
    private final BooleanSupplier slowMovement;

    public CrocodileDefaultCommand(CrocodileSubsystem subsystem, DoubleSupplier motorSpeed,
            BooleanSupplier slowMovement) {
        this.subsystem = subsystem;
        this.motorSpeed = motorSpeed;
        this.slowMovement = slowMovement;
        addRequirements(subsystem);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        //If we are holding the slow button, run the crocodile at 40% speed
        if (slowMovement.getAsBoolean()) {
            subsystem.setMotorSpeed(.4);
        } 
        //If we are not holding the slow button, run the crocodile at the speed of the joystick
        else if (Math.abs(motorSpeed.getAsDouble()) > 0.05){
            subsystem.setMotorSpeed(motorSpeed.getAsDouble());
        }
        //If we are not holding the slow button and the joystick is not being moved, stop the crocodile
        else {
            subsystem.setMotorSpeed(0);
        }
    }
}
