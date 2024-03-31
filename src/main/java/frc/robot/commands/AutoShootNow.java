// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.BatonSubsystem;
import frc.robot.utils.BatonState;
import frc.robot.utils.Globals;

public class AutoShootNow extends Command {
  BatonSubsystem baton;
  double tiltAngle = 0;
  double shooterSpeed = 0;

  /** Creates a new Shoot. */
  public AutoShootNow(BatonSubsystem baton, double tiltAngle, double shooterSpeed) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.baton = baton;
    this.tiltAngle = tiltAngle;
    this.shooterSpeed = shooterSpeed;
    addRequirements(baton);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //Globals.setSpeakerTracking(true);
    baton.setManualTiltAngle(tiltAngle);
    baton.setManualShooterSpeed(shooterSpeed);
    baton.setManualShooting(true);
    baton.setState(BatonState.AUTO_SHOOT_FAST);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    baton.setManualShooting(false);
    Globals.setSpeakerTracking(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (baton.getState() == BatonState.IDLE);
  }
}