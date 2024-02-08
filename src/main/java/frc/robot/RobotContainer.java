// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.BatonSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
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
  public final DriveSubsystem  m_robotDrive    = new DriveSubsystem(m_driverController, m_copilot_1, m_copilot_2);
  public final BatonSubsystem m_baton = new BatonSubsystem(m_driverController, m_copilot_1, m_copilot_2);

  private final SendableChooser<Command> autoChooser;

  
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

        // Register named commands
    NamedCommands.registerCommand("Shoot", Commands.print("Shooting"));
    
    // Configure the button bindings
    configureButtonBindings();

    autoChooser = AutoBuilder.buildAutoChooser(); // Default auto will be `Commands.none()`
    SmartDashboard.putData("Auto Mode", autoChooser);

    // Configure default commands
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> m_robotDrive.drive(),
            m_robotDrive));

  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
    new JoystickButton(m_driverController, Button.kR1.value)
        .whileTrue(new RunCommand(
            () -> m_robotDrive.setX(),
            m_robotDrive));

   new JoystickButton(m_driverController, Button.kTouchpad.value)
        .whileTrue(new RunCommand(
            () -> m_robotDrive.resetHeading(),
            m_robotDrive));

    // Add a button to run the example auto to SmartDashboard, this will also be in the auto chooser built above
    SmartDashboard.putData("ScoreSingle", new PathPlannerAuto("ScoreSingle"));
    SmartDashboard.putData("ScoreDouble", new PathPlannerAuto("ScoreDouble"));
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
