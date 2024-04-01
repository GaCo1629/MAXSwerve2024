// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;

public class AutoTurnToHeading extends Command {
  DriveSubsystem robotDrive;
  Timer          turnTimer = new Timer();
  double         setpointRad;
  double         timeout;

  /** Creates a new command. */
  public AutoTurnToHeading(DriveSubsystem robotDrive, double headingDeg, double timeout) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.robotDrive = robotDrive;
    this.setpointRad = Math.toRadians(headingDeg);
    this.timeout = timeout;
    addRequirements(robotDrive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Mirror setpoint direction if we are red.    
    if (DriverStation.getAlliance().isPresent() && (DriverStation.getAlliance().get() == Alliance.Red)){
      setpointRad = Math.PI - setpointRad;
    }

    // send the new setpoint and start timeout timer.
    robotDrive.newHeadingSetpoint(setpointRad);
    turnTimer.restart();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  
    SmartDashboard.putString("Mode", "Turn To Heading")  ;

    // PID Yaw control.
    double rotate = robotDrive.headingLockCalculate() * 0.80;
    if (Math.abs(rotate) < 0.025) {
      rotate = 0;
    } 

    // Convert the commanded speeds into the correct units for the drivetrain
    double rotRPS    = rotate * DriveConstants.kMaxAngularSpeed;

    // Send required power to swerve drives
    robotDrive.driveRobot(0, 0, rotRPS, false);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
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