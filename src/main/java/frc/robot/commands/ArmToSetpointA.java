// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

<<<<<<< HEAD
import edu.wpi.first.math.controller.PIDController;
=======
>>>>>>> 2b3ee9b70f2bf007028e2860c3a1fe60ea0769b2
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;

/** An example command that uses an example subsystem. */
public class ArmToSetpointA extends CommandBase {

  private final ArmSubsystem subsystem;

  public ArmToSetpointA(ArmSubsystem subsystem) {
    this.subsystem = subsystem;
    addRequirements(subsystem);
  }

  public void initialize() {
    // TODO: Set to actual position on the robot.
    subsystem.setSetpoint(0);
  }

  public boolean isFinished() {
    return subsystem.atSetpoint();
  }
}
