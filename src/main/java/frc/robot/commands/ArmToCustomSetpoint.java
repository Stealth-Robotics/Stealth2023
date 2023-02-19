// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;

/** An example command that uses an example subsystem. */
public class ArmToCustomSetpoint extends CommandBase {

  private final PIDController pid;
  private final ArmSubsystem subsystem;
  private final double setpoint;
  
  public ArmToCustomSetpoint(ArmSubsystem subsystem, double setpoint) {
    this.subsystem = subsystem;
    pid = new PIDController(0, 0, 0);
    pid.setTolerance(25);
    this.setpoint = setpoint;
    addRequirements(subsystem);
  }

  public void initialize() {
    //TODO: Set to actual position on the robot.
    pid.setSetpoint(setpoint);
  }

  public void execute() {
    subsystem.setMotorPower(pid.calculate(subsystem.getEncoderValue()));
  }

  public boolean isFinished() {
    return pid.atSetpoint();
  }
}
