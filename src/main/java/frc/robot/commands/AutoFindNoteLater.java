// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.utils.FrontImageSource;
import frc.robot.utils.Globals;
import frc.robot.utils.LEDmode;

public class AutoFindNoteLater extends Command {
  Timer      downTimer = new Timer();
  VisionSubsystem vision;
  boolean    keepLooking;
  int        counter = 0;

  public AutoFindNoteLater(VisionSubsystem vision) {
    this.vision = vision; 
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    VisionSubsystem.setFrontImageSource(FrontImageSource.NOTE);
    downTimer.restart();
    vision.flushNoteTargets();
    Globals.startNoteFinding = false;
    Globals.noteFoundInAuto = true;  // just in case looking is never enabled.
    keepLooking = false;
    counter = 0;
    SmartDashboard.putString("Mode", "Path Later");
    SmartDashboard.putString("FindNote", "Later");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // Keep resetting the timer and flushing note-target-buffer while Baton is lowering.
    if (!Globals.batonIsDown) {
      downTimer.reset();
      vision.flushNoteTargets();
    }

    SmartDashboard.putBoolean("Start Looking", Globals.startNoteFinding);
    SmartDashboard.putString("Mode", String.format("Path Later %d", counter++));
    
    if (Globals.startNoteFinding) {
      keepLooking = true;
      Globals.startNoteFinding = false;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
        Globals.startNoteFinding = false;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    boolean finished = false;

    SmartDashboard.putBoolean("Start Looking", Globals.startNoteFinding);

    if (downTimer.hasElapsed(0.1) && (keepLooking)){
      Globals.setLEDMode(LEDmode.DONE_WAITING);

      if (Globals.noteTarget.valid){
        Globals.startNoteFinding = false;
        Globals.noteFoundInAuto = true;
        finished = true;
      } else {
        Globals.noteFoundInAuto = false;
      }
    } else {
      Globals.setLEDMode(LEDmode.WAITING);
    }
    return finished;
  }
}
