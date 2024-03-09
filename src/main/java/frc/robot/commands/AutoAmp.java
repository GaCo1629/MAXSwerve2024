// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.BatonSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.utils.BatonState;
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
    baton.setState(BatonState.HOLDING);
    baton.amplify();  // start the scoring process
   }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Read baton sensors
    robotDrive.driveAutoAmplify();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Globals.setAmplifying(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // wailt till note is picked up, but don't wait too long
    return ((baton.getState() == BatonState.IDLE) );
  }

}