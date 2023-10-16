package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Swerve.DrivebaseSubsystem;

public class LevelRobot extends CommandBase {
    // Constants for the PID
    private static final double PID_kP = 0.5;
    private static final double PID_kI = 0.001;
    private static final double PID_kD = 0.05;

    private Timer timer;
    boolean disableExecute = false;
    // Dont allow it to go faster than 70% motor speed
    private static final double LEVELING_DRIVE_SPEED_LIMIT = 0.6;

    private final DrivebaseSubsystem drive;
    // Construct the PID controller
    private final PIDController pid = new PIDController(
            PID_kP,
            PID_kI,
            PID_kD);

    public LevelRobot(DrivebaseSubsystem drivetrain) {
        drive = drivetrain;
        addRequirements(drive);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {

        timer = new Timer();

        pid.setTolerance(0.5);
        pid.setSetpoint(0);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // Get the pitch and the roll from the gyro
        double roll = drive.getRollAsDouble();
        double pitch = drive.getPitchAsDouble();
        // Calculate the movement based on the sum of the pitch and roll, that way it
        // will zero both
        double calculationMovement = pid.calculate(pitch + roll, 0);
        // Clamp the movement to the max speed
        calculationMovement = MathUtil.clamp(calculationMovement, -LEVELING_DRIVE_SPEED_LIMIT,
                LEVELING_DRIVE_SPEED_LIMIT);
        // Set the speed based on the calculation
        if(!disableExecute){
            drive.drive(new Translation2d(-calculationMovement, 0), 0, false, true);
        }
    }

    @Override
    public void end(boolean interrupted) {
        // Stop the robot
        drive.drive(new Translation2d(0, 0), 0, false, true);
    }

    @Override
    public boolean isFinished() {
        // Stop if we are at the setpoint
        if(pid.atSetpoint())
        {
            timer.start();
            //drive.drive(new Translation2d(0, 0), 0, false, true);
            disableExecute = true;
            
        }

        else {
            timer.reset();
            disableExecute = false;
        }

        if(pid.atSetpoint() && timer.get() >= 2)
        {
            return true;
        }
        
        return false;
        

    }

}