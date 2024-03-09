// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.utils.Globals;

public class AutoFindNote extends Command {
  Timer      downTimer = new Timer();
  VisionSubsystem vision;

  /** Creates a new AutoFindNote. */
  public AutoFindNote(VisionSubsystem vision) {
    this.vision = vision;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    downTimer.restart();
    vision.flushNoteTargets();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // Keep resetting the timer and flushing note-target-buffer while Baton is lowering.
    if (!Globals.batonIsDown) {
      downTimer.reset();
      vision.flushNoteTargets();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (downTimer.hasElapsed(0.1) && Globals.noteTarget.valid);
  }
}
