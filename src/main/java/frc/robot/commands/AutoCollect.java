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

public class AutoCollect extends Command {
  BatonSubsystem baton;
  DriveSubsystem robotDrive;
  Timer          collectTimer = new Timer();

  /** Creates a new Shoot. */
  public AutoCollect(BatonSubsystem baton, DriveSubsystem robotDrive) {
    this.baton = baton;
    this.robotDrive = robotDrive;
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
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double xSpeed = BatonConstants.noteApproachSpeed;
    double rotate = 0;
    SmartDashboard.putString("Mode", "Auto Collect")  ;

    if (baton.getState() == BatonState.COLLECTING){
      if (Globals.noteTarget.valid){
        // Calculate turn power to point to note.
        rotate = robotDrive.trackingCalculate(Globals.noteTarget.bearingDeg);
        if (Math.abs(Globals.noteTarget.bearingDeg) < 10){
          xSpeed = Globals.noteTarget.range * 0.35; 
        }
        robotDrive.lockCurrentHeading(); 
      } else {
        rotate = robotDrive.headingLockCalculate();
      }
    }

    // Convert the commanded speeds into the correct units for the drivetrain
    double xSpeedMPS = xSpeed * DriveConstants.kMaxSpeedMetersPerSecond;
    double rotRPS    = rotate * DriveConstants.kMaxAngularSpeed;

    // prepare for return to heading hold & Send power to swerve modules
    robotDrive.driveRobot(xSpeedMPS, 0, rotRPS, false);

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