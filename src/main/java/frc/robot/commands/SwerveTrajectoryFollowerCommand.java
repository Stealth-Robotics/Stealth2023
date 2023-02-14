package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants;
import frc.robot.Constants.Swerve;

public class SwerveTrajectoryFollowerCommand extends CommandBase{

    private final ProfiledPIDController thetaController;
    private final Swerve drivetrain;
    private final Trajectory trajectory;
    private Timer m_timer = new Timer();
    
    public SwerveTrajectoryFollowerCommand(Trajectory traj, Swerve drivetrain) {
        thetaController = new ProfiledPIDController(
            Constants.AutoConstants.kPThetaController, 0, 0, 
            Constants.AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        trajectory = traj;
        this.drivetrain = drivetrain;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements();
      }
    
      // Called when the command is initially scheduled.
      @Override
      public void initialize() {
        m_timer.reset();
        m_timer.start();
      }
    
      // Called every time the scheduler runs while the command is scheduled.
      @Override
      public void execute() {/*
        double curTime = m_timer.get();
        var desiredState = trajectory.sample(curTime);
    
        var targetChassisSpeeds =
            thetaController.calculate(m_pose.get(), desiredState, m_desiredRotation.get());
        var targetModuleStates = m_kinematics.toSwerveModuleStates(targetChassisSpeeds);
    
        m_outputModuleStates.accept(targetModuleStates);*/
      }
    
      // Called once the command ends or is interrupted.
      @Override
      public void end(boolean interrupted) {}
    
      // Returns true when the command should end.
      @Override
      public boolean isFinished() {
        return false;
      }
}
