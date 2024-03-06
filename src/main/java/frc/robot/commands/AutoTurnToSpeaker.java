// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.BatonSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.utils.Target;

public class AutoTurnToSpeaker extends Command {
  DriveSubsystem robotDrive;
  BatonSubsystem baton;
  Timer          turnTimer = new Timer();
  double         setpointRad;
  double         timeout;

  /** Creates a new command. */
  public AutoTurnToSpeaker(BatonSubsystem baton, DriveSubsystem robotDrive, double timeout) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.baton      = baton;
    this.robotDrive = robotDrive;
    this.timeout    = timeout;
    addRequirements(robotDrive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    // start timeout timer.
    turnTimer.restart();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Let drive turn to new heading.  Disable target tracking if we are rasing baton
    Target speaker = robotDrive.getTargetFromOdometry();
    if (speaker.valid) {
      robotDrive.newHeadingSetpoint(Math.toRadians(speaker.bearingDeg));
      baton.setTiltAngle(baton.rangeToAngle(speaker.range) - 6 ); 
      baton.setShooterRPM(baton.rangeToRPM(speaker.range));
    }
    
    robotDrive.driveAutoTurnToHeading();
  }


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    baton.setTiltAngle(0); 
    baton.setShooterRPM(0);
    turnTimer.stop();
  }

  /**
   * Signal finished if we have reached the setpoint, or if timeout has expired.
   * A timeout of 0 implies no timeout
   */
  @Override
  public boolean isFinished() {
    return (robotDrive.atSetpoint() || ((timeout) > 0 && turnTimer.hasElapsed(timeout)));
  }

}