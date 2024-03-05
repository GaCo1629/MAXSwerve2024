// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.BatonState;
import frc.robot.subsystems.BatonSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Globals;

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
    Globals.setSpeakerTracking(true);
    baton.setState(BatonState.AUTO_SHOOT);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Read baton sensors
    robotDrive.driveTelep();
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