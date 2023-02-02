package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import frc.robot.RobotMap;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Rotation extends TrapezoidProfileSubsystem {
   private final WPI_TalonFX rotationMotor;
   private final PIDController rotationController;

   public int targetPostition = 0;

   private final ArmFeedforward m_feedforward = new ArmFeedforward(
         0, 0, 0, 0);

   public Rotation() {

      super(
         new TrapezoidProfile.Constraints(
            0, 0),
        0);
    
      rotationMotor = new WPI_TalonFX(RobotMap.ArmHardware.ROTATION_MOTOR);

      rotationMotor.setNeutralMode(NeutralMode.Brake);
      rotationController = new PIDController(0, 0, 0);
      
   }
     

    public Command setRotationPosition(double kArmOffsetRads) {
      return Commands.runOnce(() -> setGoal(kArmOffsetRads), this);
    }


   @Override
   public void useState(TrapezoidProfile.State setpoint) {
      // Calculate the feedforward from the sepoint
      double feedforward = m_feedforward.calculate(setpoint.position, setpoint.velocity);
      // Add the feedforward to the PID output to get the motor output
      rotationMotor.setSetpoint(
          setpoint.kPosition, setpoint.position, feedforward / 12.0);
    }
  
   
  }
