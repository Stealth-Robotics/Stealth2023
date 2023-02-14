// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Claw;

public class ToggleClawCommand extends CommandBase {
  private final Claw claw;

  public ToggleClawCommand(Claw claw) {
    this.claw = claw;
    addRequirements(claw);
}

  public void initialize() {
    claw.toggleClaw();
  }
  
  public boolean isFinished() {
    return true;
  }

}
