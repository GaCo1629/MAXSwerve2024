// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.BatonConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.BatonSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.utils.BatonState;
import frc.robot.utils.FrontImageSource;
import frc.robot.utils.Globals;

public class AutoCollectSmart extends Command {
  BatonSubsystem baton;
  DriveSubsystem robotDrive;
  double         turnToRad;
  boolean        seekingNote;
  Timer          collectTimer = new Timer();
  Timer          seekTimer    = new Timer();

  /** Creates a new Shoot. */
  public AutoCollectSmart(BatonSubsystem baton, DriveSubsystem robotDrive, double turnToDeg) {
    this.baton = baton;
    this.robotDrive = robotDrive;
    this.turnToRad = Math.toRadians(turnToDeg);
    seekingNote = false;
    addRequirements(baton, robotDrive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    VisionSubsystem.setFrontImageSource(FrontImageSource.NOTE);
    Globals.setNoteTracking(true);
    robotDrive.lockCurrentHeading();
    baton.collect();
    collectTimer.restart();
    seekTimer.restart();

    if (!Globals.noteFoundInAuto) {
      turnToNewHeading();      
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double xSpeed = BatonConstants.noteApproachSpeed;
    double rotate = 0;
    SmartDashboard.putString("Mode", "Smart Collect")  ;

    if (baton.getState() == BatonState.COLLECTING){
      if (seekingNote) {
        // we are turning to new hopefull note.
        rotate = robotDrive.headingLockCalculate() * 0.60;  // PID Yaw control.
        xSpeed = 0;

        // are we there yet?
        if (robotDrive.atSetpoint()) {
          seekingNote = false;
          seekTimer.restart();
          collectTimer.restart();
        }
        
      } else {
        // we are trying to collect as requested.
        if (Globals.noteTarget.valid){
          // we see note
          seekTimer.restart();

          rotate = robotDrive.trackingCalculate(Globals.noteTarget.bearingDeg);  // Calculate turn power to point to note.
          if (Math.abs(Globals.noteTarget.bearingDeg) < 10){
            xSpeed = Globals.noteTarget.range * 0.35; 
          }
          robotDrive.lockCurrentHeading(); 
        } else {
          // we can't see the target. wait just a short moment 
          rotate = robotDrive.headingLockCalculate();
          if (seekTimer.hasElapsed(1.0)) {
            turnToNewHeading();      
          }
        }
      }
    }

    // Convert the commanded speeds into the correct units for the drivetrain
    double xSpeedMPS = xSpeed * DriveConstants.kMaxSpeedMetersPerSecond;
    double rotateRPS = rotate * DriveConstants.kMaxAngularSpeed;

    // Send power to swerve modules
    robotDrive.driveRobot(xSpeedMPS, 0, rotateRPS, false);
  }

  
  private void turnToNewHeading() {
    robotDrive.newHeadingSetpoint(turnToRad);      
    seekingNote = true;
  }


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    baton.stopIntake();  // Skip this to speed up action after collection.. also stops note tracking
    Globals.setNoteTracking(false);
    collectTimer.stop();
    robotDrive.stopRobot();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // wailt till note is picked up, but don't wait too long
    return ((baton.getState() == BatonState.HOLDING) || (collectTimer.hasElapsed(3.0)));
  }

}