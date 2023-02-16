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
  private final DoubleSupplier joystick;
  private final ArmSubsystem telescope;
  private final BooleanSupplier intake;

  public ArmDefaultCommand(ArmSubsystem telescope, DoubleSupplier joystick, BooleanSupplier intake) {
    this.joystick = joystick;
    this.telescope = telescope;
    this.intake = intake;
    addRequirements(telescope);
  }

  @Override
  public void execute() {
    if (intake.getAsBoolean()) {
      telescope.setSetpoint(MathUtil.clamp(
          (telescope.getSetpoint() + joystick.getAsDouble()),
          Constants.TelescopeConstants.LOWER_BOUND_INTAKE_OUT_TICKS, Constants.TelescopeConstants.UPPER_BOUND_TICKS));
    } else {
      telescope.setSetpoint(MathUtil.clamp(
          (telescope.getSetpoint() + joystick.getAsDouble()),
          Constants.TelescopeConstants.LOWER_BOUND_INTAKE_IN_TICKS, Constants.TelescopeConstants.UPPER_BOUND_TICKS));
    }

  }
}
