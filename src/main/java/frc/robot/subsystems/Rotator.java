package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import frc.robot.Constants;
import frc.robot.RobotMap;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;

public class Rotator extends ProfiledPIDSubsystem {
   private final WPI_TalonFX rotationMotor;
   private final DutyCycleEncoder encoder;

   private final ArmFeedforward feedforward;

   public Rotator() {
      super(
            new ProfiledPIDController(
                  Constants.ArmConstants.ARM_P_COEFF,
                  Constants.ArmConstants.ARM_I_COEFF,
                  Constants.ArmConstants.ARM_D_COEFF,
                  new TrapezoidProfile.Constraints(
                        Constants.ArmConstants.MAX_VELOCITY,
                        Constants.ArmConstants.MAX_ACCELERATION)),
            0);

      rotationMotor = new WPI_TalonFX(RobotMap.Arm.ROTATOR_MOTOR);
      rotationMotor.setNeutralMode(NeutralMode.Brake);

      encoder = new DutyCycleEncoder(RobotMap.Arm.ENCODER_PORT);

      feedforward = new ArmFeedforward(
            Constants.ArmConstants.ARM_KS_COEFF,
            Constants.ArmConstants.ARM_KG_COEFF,
            Constants.ArmConstants.ARM_KV_COEFF,
            Constants.ArmConstants.ARM_KA_COEFF);

      encoder.setDistancePerRotation(getMeasurement());
   }

   public void setSpeed(double speed) {
      rotationMotor.set(ControlMode.PercentOutput, speed);
   }

   @Override
   protected void useOutput(double output, State setpoint) {
      rotationMotor.set(
            ControlMode.Current,
            output + (feedforward.calculate(setpoint.position, setpoint.velocity)));
   }

   @Override
   protected double getMeasurement() {
      return Math.toRadians((encoder.getAbsolutePosition()
            * (360 * Constants.ArmConstants.GEAR_RATIO / Constants.ArmConstants.ENCODER_CPR))
            + Constants.ArmConstants.ENCODER_OFFSET);
   }

   @Override
   public void periodic() {
      super.periodic();

      // TODO: Test If Accurate After Entering Constants
      System.out.println("Current Arm Degrees: " + (encoder.getAbsolutePosition()
            * (360 * Constants.ArmConstants.GEAR_RATIO / Constants.ArmConstants.ENCODER_CPR))
            + Constants.ArmConstants.ENCODER_OFFSET);
   }
}
