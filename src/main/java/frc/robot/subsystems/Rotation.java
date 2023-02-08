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

   public Rotation() {
      rotationMotor = new WPI_TalonFX(RobotMap.ArmHardware.ROTATION_MOTOR);
      rotationMotor.setNeutralMode(NeutralMode.Brake);
      rotationController = new PIDController(0, 0, 0);
      feedforward = new ArmFeedforward(
            0, 0, 0, 0);
           
   }
   public void setSetpoint(double setpoint)
   {
      rotationController.setSetpoint(setpoint);
   }
   
   public double getRotationPositionRadians()
   {
      //convert this from raw sensor unit to radians
      return Math.toRadians(rotationMotor.getSelectedSensorPosition());
   }

   public void updatePosition() {
      rotationMotor.setVoltage(feedforward.calculate(getRotationPositionRadians(), 0.1)
            + rotationController.calculate(getRotationPositionRadians()));
   }

}
