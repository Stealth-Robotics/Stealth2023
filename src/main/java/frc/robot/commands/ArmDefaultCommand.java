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
  private long startLoop, endLoop;
  private final DoubleSupplier joystick;
  private final ArmSubsystem subsystem;
  
  public ArmDefaultCommand(ArmSubsystem subsystem, DoubleSupplier joystick) {
    pid = new PIDController(0, 0, 0);
    this.joystick = joystick;
    this.subsystem =subsystem;
    addRequirements(subsystem);
  }
  @Override
  public void initialize(){
    //gets time in milliseconds
    startLoop = System.nanoTime() / (long)Math.pow(10, 6);
  }
  @Override
  public void execute() {
    endLoop = System.nanoTime() / (long)Math.pow(10, 6);
    long deltaTime = endLoop - startLoop;
    //TODO: Make constant, 1 is placeholder for ticks per millisecond
    pid.setSetpoint(joystick.getAsDouble() * deltaTime * 1);
    subsystem.setMotorPower(pid.calculate(subsystem.getEncoderValue()));
    startLoop = System.nanoTime() / (long)Math.pow(10, 6);
  }
  @Override
  public boolean isFinished(){
    return pid.atSetpoint();
  }

}
