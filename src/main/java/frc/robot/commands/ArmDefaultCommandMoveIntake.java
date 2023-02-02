// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUsageId;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.Intake;

/** An example command that uses an example subsystem. */
public class ArmDefaultCommandMoveIntake extends CommandBase {

  private final PIDController pid;
  private long endLoop;
  private final DoubleSupplier joystick;
  private final ArmSubsystem subsystem;
  private final BooleanSupplier intakeOut;
  private final Intake intake;

  public ArmDefaultCommandMoveIntake(ArmSubsystem subsystem, DoubleSupplier joystick, BooleanSupplier intakeOut,
      Intake intake) {
    pid = new PIDController(0, 0, 0);
    this.joystick = joystick;
    this.subsystem = subsystem;
    this.intakeOut = intakeOut;
    this.intake = intake;
    addRequirements(subsystem);
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
    long deltaTime = endLoop - temp;
    // if driver tries to move arm to where it will break the robot, deploy intake
    // before setting setpoint
    if ((pid.getSetpoint() + (joystick.getAsDouble() * deltaTime
        * Constants.ArmConstants.TICKS_PER_SECOND)) < Constants.ArmConstants.LOWER_BOUND_INTAKE_IN_TICKS
        && !intake.isIntakeOut()) {
      new InstantCommand(() -> intake.deployIntake());
    }
    // wait half a second after retract intake to set pid setpoint to avoid killing
    // the robot
    new SequentialCommandGroup(
        new WaitCommand(0.5),
        new InstantCommand(() -> pid.setSetpoint(MathUtil.clamp(
            (pid.getSetpoint() + (joystick.getAsDouble() * deltaTime * Constants.ArmConstants.TICKS_PER_SECOND)),
            Constants.ArmConstants.LOWER_BOUND_INTAKE_OUT_TICKS, Constants.ArmConstants.UPPER_BOUND_TICKS))));

    // after arm moves out of the way, wait half a second before retracting the
    // intake for safety
    if (subsystem.getEncoderValue() > Constants.ArmConstants.LOWER_BOUND_INTAKE_IN_TICKS) {
      new SequentialCommandGroup(
          new WaitCommand(0.5),
          new InstantCommand(() -> intake.retractIntake()));
    }

    subsystem.setMotorPower(pid.calculate(subsystem.getEncoderValue()));
  }
}
