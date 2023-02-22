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
                  Constants.RotatorConstants.ROTATOR_P_COEFF,
                  Constants.RotatorConstants.ROTATOR_I_COEFF,
                  Constants.RotatorConstants.ROTATOR_D_COEFF,
                  new TrapezoidProfile.Constraints(
                        Constants.RotatorConstants.MAX_VELOCITY,
                        Constants.RotatorConstants.MAX_ACCELERATION)),
            0);

      rotationMotor = new WPI_TalonFX(RobotMap.Rotator.ROTATOR_MOTOR);
      rotationMotor.setNeutralMode(NeutralMode.Brake);

      encoder = new DutyCycleEncoder(RobotMap.Rotator.ENCODER_PORT);

      feedforward = new ArmFeedforward(
            Constants.RotatorConstants.ROTATOR_KS_COEFF,
            Constants.RotatorConstants.ROTATOR_KG_COEFF,
            Constants.RotatorConstants.ROTATOR_KV_COEFF,
            Constants.RotatorConstants.ROTATOR_KA_COEFF);

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
      System.out.println(output + (feedforward.calculate(setpoint.position, setpoint.velocity)));
   }

   @Override
   protected double getMeasurement() {
      return Math.toRadians((encoder.getAbsolutePosition() * 360) - Constants.RotatorConstants.ENCODER_OFFSET);
   }

   @Override
   public void setGoal(double goal){
      super.setGoal(goal);
      super.enable();
   }
   public double getSetpoint(){
      return super.getController().getSetpoint().position;
   }

   @Override
   public void periodic() {
      super.periodic();
      
      // TODO: Test If Accurate
      //System.out.println("Current Rotation Degrees: " + ((encoder.getAbsolutePosition() * 360) - Constants.ArmConstants.ENCODER_OFFSET));
   }
}
