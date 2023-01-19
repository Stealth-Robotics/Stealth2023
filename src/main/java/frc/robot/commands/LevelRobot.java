package frc.robot.commands;

import com.ctre.phoenix.sensors.WPI_Pigeon2;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ExampleSubsystem;

public class LevelRobot extends CommandBase{
    private final PIDController pid = new PIDController(10, 0, 0);
    private final WPI_Pigeon2 pigeon = new WPI_Pigeon2(2);
    public LevelRobot(){
    }
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        pid.setTolerance(5);
        pid.setSetpoint(0);
        System.out.println("Command Init");
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        System.out.println(pid.calculate(pigeon.getPitch(), 0));
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return pid.atSetpoint();
    }

}
