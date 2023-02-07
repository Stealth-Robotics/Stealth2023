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

   public void setPosition(double setpoint) {
      rotationMotor.setVoltage(feedforward.calculate(rotationMotor.getSelectedSensorPosition(),0.1)
            + rotationController.calculate(setpoint));
   }

}
