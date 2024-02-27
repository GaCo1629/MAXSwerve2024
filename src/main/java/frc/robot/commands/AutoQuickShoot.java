// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.BatonState;
import frc.robot.subsystems.BatonSubsystem;
import frc.robot.subsystems.Globals;
import frc.robot.subsystems.LEDmode;

public class AutoQuickShoot extends Command {
  BatonSubsystem baton;


  /** Creates a new Shoot. */
  public AutoQuickShoot(BatonSubsystem baton) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.baton = baton;
    addRequirements(baton);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    baton.setState(BatonState.AUTO_SHOOT);
    Globals.setLEDMode(LEDmode.SHOOTING);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Globals.setLEDMode(LEDmode.SPEEDOMETER);
    baton.quickShootingOff();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    return (baton.getState() == BatonState.IDLE);
  }

}