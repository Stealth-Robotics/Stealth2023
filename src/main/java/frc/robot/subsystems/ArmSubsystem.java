// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase; 

public class ArmSubsystem extends SubsystemBase {

  private final WPI_TalonFX armMotor;

  public ArmSubsystem() {
    armMotor = new WPI_TalonFX(0);
    armMotor.setNeutralMode(NeutralMode.Brake); 
    armMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
  }

  public void setMotorPower(double motorPower){
    armMotor.set(ControlMode.PercentOutput, motorPower);
  }

  public double getEncoderValue() {
    return armMotor.getSelectedSensorPosition();
  }
}
