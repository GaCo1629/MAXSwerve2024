// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
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

public class AutoAmp extends Command {
  BatonSubsystem baton;
  DriveSubsystem robotDrive;

  /** Creates a new Auto Amp. */
  public AutoAmp(BatonSubsystem baton, DriveSubsystem robotDrive) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.baton = baton;
    this.robotDrive = robotDrive;
    addRequirements(baton, robotDrive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    VisionSubsystem.setFrontImageSource(FrontImageSource.AMP);
    baton.amplify(true);  // start the scoring process
   }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double xSpeed = 0;
    double ySpeed = 0;
    double rotate = 0;

    SmartDashboard.putString("Mode", "Amplify")  ;
   
    // If we can see the amp. try to get directly in front of it.
    if (Globals.ampTarget.valid) {
      // If we are coming in, use the heading error (from 90) to strafe.
      // once we are close, use the angle error 
      if (Globals.ampTarget.range > 0.3) {
        // Point to amp and strafe sideways to get to point to 90 (centered on target)
        xSpeed = (Globals.ampTarget.range * 0.25);
        ySpeed = (robotDrive.getHeadingDeg() - 90) * 0.00556;
        rotate = robotDrive.trackingCalculate(Globals.ampTarget.bearingDeg); 
      } else {
        // Point to 90 degrees and strafe sideways to get the tag centered
        robotDrive.newHeadingSetpoint(Math.PI / 2);
        xSpeed = BatonConstants.amplifierApproachSpeed;
        ySpeed = Globals.ampTarget.bearingDeg * -0.02;
        rotate = robotDrive.headingLockCalculate();
      }

      xSpeed = MathUtil.clamp(xSpeed, 0, 0.4);
      ySpeed = MathUtil.clamp(ySpeed, -0.2, 0.2);
    }
    
    double xSpeedMPS = xSpeed * DriveConstants.kMaxSpeedMetersPerSecond;
    double ySpeedMPS = ySpeed * DriveConstants.kMaxSpeedMetersPerSecond;
    double rotRPS    = rotate * DriveConstants.kMaxAngularSpeed;

    // prepare for return to heading hold & Send power to swerve modules
    robotDrive.lockCurrentHeading();  
    robotDrive.driveRobot(xSpeedMPS, ySpeedMPS, rotRPS, false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
   VisionSubsystem.setFrontImageSource(FrontImageSource.NOTE);
   Globals.setAmplifying(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // wailt till note is picked up, but don't wait too long
    return ((baton.getState() == BatonState.IDLE) );
  }

}