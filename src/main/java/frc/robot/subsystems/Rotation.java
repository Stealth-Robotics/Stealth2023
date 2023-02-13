package frc.robot.subsystems;


import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import frc.robot.RobotMap;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Rotation extends SubsystemBase {
   private final WPI_TalonFX rotationMotor;
   private final PIDController rotationController;
   private final ArmFeedforward feedforward;

   private final double ticksPerRev = 2048;
   private final double steering_reduction = 1;
   private final double positionCoefficient =  2.0 * Math.PI / ticksPerRev * steering_reduction;
 
  
   public Rotation() {
      rotationMotor = new WPI_TalonFX(RobotMap.ArmHardware.ROTATION_MOTOR);
      rotationMotor.setNeutralMode(NeutralMode.Brake);
      rotationController = new PIDController(1, 0, 0);
      feedforward = new ArmFeedforward(
            1, 1, 0.5, 0.1);
       
           
   }
   public void setSetpoint(double setpoint)
   {
      rotationController.setSetpoint(setpoint);
   }
   
   public double getRotationPositionRadians()
   {
      //convert this from raw sensor unit to radians
      return (rotationMotor.getSelectedSensorPosition() * positionCoefficient);
   }

   public void updatePosition() {
      rotationMotor.setVoltage(feedforward.calculate(getRotationPositionRadians(), 0.1)
            + rotationController.calculate(getRotationPositionRadians()));
   }

   public boolean atSetpoint(){
      return rotationController.atSetpoint();
   }

}
