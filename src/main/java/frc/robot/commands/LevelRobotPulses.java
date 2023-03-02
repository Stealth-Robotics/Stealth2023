package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Swerve.DrivebaseSubsystem;

public class LevelRobotPulses extends CommandBase{
    private final DrivebaseSubsystem drive;
    private double timer = 0;
    private double totalError;
    public LevelRobotPulses(DrivebaseSubsystem drivetrain){

        drive = drivetrain;
        addRequirements(drive);
        
        timer = System.currentTimeMillis();
        totalError = drive.getPitchAsDouble() + drive.getRollAsDouble();
    }
    @Override
    public void initialize() {
        
    }
    @Override
    public void execute() {
        double temp = timer;
        timer = System.currentTimeMillis();
        double delta = timer - temp; 
        totalError = drive.getPitchAsDouble() + drive.getRollAsDouble();
        double direction = -Integer.signum((int)totalError);
        if (delta > 500){
            drive.drive(new Translation2d(1 * direction, 0), 0, true, true);
        }
    }
    @Override
    public boolean isFinished() { 
        return Math.abs(totalError) < 5;
    }
    @Override
    public void end(boolean interrupted) {
        drive.drive(new Translation2d(0, 0), 0, true, true);
    }   
}