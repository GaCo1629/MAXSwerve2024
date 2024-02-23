// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.PS4Controller.Button;

import frc.robot.Constants.OIConstants;
import frc.robot.commands.Shoot;
import frc.robot.subsystems.BatonSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LiftSubsystem;
import frc.robot.subsystems.VisionSubsystem;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {


// The driver's controller
  PS4Controller m_driverController  = new PS4Controller(OIConstants.kDriverControllerPort);
  Joystick  m_copilot_1             = new Joystick(OIConstants.kCoPilotController1Port);
  Joystick  m_copilot_2             = new Joystick(OIConstants.kCoPilotController2Port);

  // The robot's subsystems
  public final DriveSubsystem  m_robotDrive     = new DriveSubsystem(m_driverController, m_copilot_1, m_copilot_2);
  public final BatonSubsystem  m_baton          = new BatonSubsystem(m_driverController, m_copilot_1, m_copilot_2);
  public final LiftSubsystem   m_lift           = new LiftSubsystem(m_driverController, m_copilot_1, m_copilot_2);
  public final VisionSubsystem m_vision         = new VisionSubsystem();

  private final SendableChooser<Command> autoChooser;

    /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

        // Register named commands
    NamedCommands.registerCommand("Shoot", new Shoot(m_baton));
    NamedCommands.registerCommand("Collector On",   m_baton.collectCmd());
    
    // Configure the button bindings
    configureButtonBindings();

    autoChooser = AutoBuilder.buildAutoChooser(); // Default auto will be `Commands.none()`
    SmartDashboard.putData("Auto Mode", autoChooser);

    // Configure default commands
    m_robotDrive.setDefaultCommand(m_robotDrive.driveCmd());

  }

  private void configureButtonBindings() {

    // Pilot Functions
    new JoystickButton(m_driverController, Button.kR1.value)    
        .onTrue(m_robotDrive.setTurboModeCmd(true))
        .onFalse(m_robotDrive.setTurboModeCmd(false));
    
    new JoystickButton(m_driverController, Button.kL1.value)    
        .onTrue(m_baton.collectCmd())
        .onTrue(m_robotDrive.setNoteTrackingCmd(true))
        .onFalse(m_baton.stopIntakeCmd())
        .onFalse(m_robotDrive.setNoteTrackingCmd(false));
        
    new JoystickButton(m_driverController, Button.kR2.value)
        .whileTrue(m_baton.fireCmd());

    new JoystickButton(m_driverController, Button.kL2.value)    
        .onTrue(m_robotDrive.setSpeakerTrackingCmd(true))
        .onFalse(m_robotDrive.setSpeakerTrackingCmd(false));

    new JoystickButton(m_driverController, Button.kTouchpad.value)
        .onTrue(m_robotDrive.resetHeadingCmd());

    new JoystickButton(m_driverController, Button.kCross.value)
        .onTrue(m_baton.stopIntakeCmd());

    // Co-Pilot Functions
    new JoystickButton(m_copilot_1, Button.kSquare.value)
        .onTrue(m_baton.ejectCmd())
        .onFalse(m_baton.stopIntakeCmd());

    new JoystickButton(m_copilot_1, Button.kTriangle.value)
        .whileTrue(m_baton.fireCmd());

    new JoystickButton(m_copilot_1, Button.kCross.value)
        .onTrue(m_baton.stopIntakeCmd());

    // Add a button to run the auto to SmartDashboard, this will also be in the auto chooser built above
    SmartDashboard.putData("ScoreSingle",  new PathPlannerAuto("ScoreSingle"));
    SmartDashboard.putData("ScoreDouble",  new PathPlannerAuto("ScoreDouble"));
    SmartDashboard.putData("ScoreTripple", new PathPlannerAuto("ScoreTripple"));
    SmartDashboard.putData("TwoOnTheWall", new PathPlannerAuto("TwoOnTheWall"));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
