// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase {

  private final WPI_TalonFX armMotor;
  private final PIDController telescopePID;

  public ArmSubsystem() {
    armMotor = new WPI_TalonFX(0);
    armMotor.setNeutralMode(NeutralMode.Brake);
    armMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    // TODO: update this with constants we tune, update tolerance
    this.telescopePID = new PIDController(0, 0, 0);
  }

  private void setMotorPower(double motorPower) {
    armMotor.set(ControlMode.PercentOutput, motorPower);
  }

  public double getEncoderValue() {
    return armMotor.getSelectedSensorPosition();
  }

  public void setSetpoint(double setpoint) {
    telescopePID.setSetpoint(setpoint);
  }

  public boolean atSetpoint() {
    return telescopePID.atSetpoint();
  }

  public double getSetpoint(){
    return telescopePID.getSetpoint();
  }

  @Override
  public void periodic() {
    System.out.println(getEncoderValue());
    setMotorPower(telescopePID.calculate(getEncoderValue()));
  }
}
