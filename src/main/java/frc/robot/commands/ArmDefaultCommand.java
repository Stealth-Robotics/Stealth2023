// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;

/** An example command that uses an example subsystem. */
public class ArmDefaultCommand extends CommandBase {

  private final PIDController pid;
  
  public ArmDefaultCommand(ArmSubsystem subsystem, DoubleSupplier doubleSupplier) {
    pid = new PIDController(0, 0, 0);
    addRequirements(subsystem);
  }

  public void execute() {
    
  }

}
