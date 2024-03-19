// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.BatonSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.utils.BackImageSource;
import frc.robot.utils.BatonState;
import frc.robot.utils.Globals;
import frc.robot.utils.LEDmode;

public class AutoShoot extends Command {
  BatonSubsystem baton;
  DriveSubsystem robotDrive;

  /** Creates a new Shoot. */
  public AutoShoot(BatonSubsystem baton, DriveSubsystem robotDrive) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.baton = baton;
    this.robotDrive = robotDrive;
    addRequirements(baton, robotDrive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    VisionSubsystem.setBackImageSource(BackImageSource.SPEAKER);
    Globals.setSpeakerTracking(true);
    baton.setState(BatonState.AUTO_SHOOT);
    robotDrive.lockCurrentHeading();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double rotate = 0;
    SmartDashboard.putString("Mode", "Auto Shoot")  ;

    if (Globals.speakerTarget.valid) {
      Globals.setLEDMode(LEDmode.SPEAKER_DETECTED);

      // Calculate turn power to point to speaker.
      rotate = robotDrive.trackingCalculate(Globals.speakerTarget.bearingDeg);
      robotDrive.lockCurrentHeading();  // prepare for return to heading hold

    } else if (Globals.odoTarget.valid) {
      Globals.setLEDMode(LEDmode.SEEKING);

      rotate = robotDrive.trackingCalculate(robotDrive.getHeadingDeg() - Globals.odoTarget.bearingDeg);
      rotate = MathUtil.clamp(rotate, -0.4, 0.4);
      robotDrive.lockCurrentHeading();  // prepare for return to heading hold
    }
    
    // Convert the commanded speeds into the correct units for the drivetrain
    double rotRPS    = rotate * DriveConstants.kMaxAngularSpeed;

    // Send required power to swerve drives
    robotDrive.driveRobot(0, 0, rotRPS, false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Globals.setSpeakerTracking(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (baton.getState() == BatonState.IDLE);
  }

}