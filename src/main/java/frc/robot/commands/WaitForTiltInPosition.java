// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.BatonSubsystem;

public class WaitForTiltInPosition extends Command {
  BatonSubsystem baton;

  /** Creates a new WaitForTiltInPosition. */
  public WaitForTiltInPosition(BatonSubsystem baton) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.baton = baton;
    addRequirements(baton);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return baton.tiltIsInPosition();
  }
}
