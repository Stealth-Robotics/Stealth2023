// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;

/** An example command that uses an example subsystem. */
public class ArmDefaultCommand extends CommandBase {

  private final PIDController pid;
  private long endLoop;
  private final DoubleSupplier joystick;
  private final ArmSubsystem telescope;
  private final BooleanSupplier intake;

  public ArmDefaultCommand(ArmSubsystem telescope, DoubleSupplier joystick, BooleanSupplier intake) {
    pid = new PIDController(0, 0, 0);
    this.joystick = joystick;
    this.telescope = telescope;
    this.intake = intake;
    addRequirements(telescope);
  }

  @Override
  public void initialize() {
    // gets time in seconds
    endLoop = System.nanoTime() / (long) Math.pow(10, 9);
  }

  @Override
  public void execute() {
    long temp = endLoop;
    // gets end of loop time in seconds to find deltaTime
    endLoop = System.nanoTime() / (long) Math.pow(10, 9);
    double deltaTime = endLoop - temp;
    if (intake.getAsBoolean()) {
      telescope.setSetpoint(MathUtil.clamp(
          (telescope.getSetpoint() + (joystick.getAsDouble() * deltaTime * Constants.ArmConstants.TICKS_PER_SECOND)),
          Constants.ArmConstants.LOWER_BOUND_INTAKE_OUT_TICKS, Constants.ArmConstants.UPPER_BOUND_TICKS));
    } else {
      telescope.setSetpoint(MathUtil.clamp(
          (telescope.getSetpoint() + (joystick.getAsDouble() * deltaTime * Constants.ArmConstants.TICKS_PER_SECOND)),
          Constants.ArmConstants.LOWER_BOUND_INTAKE_IN_TICKS, Constants.ArmConstants.UPPER_BOUND_TICKS));
    }

  }
}
