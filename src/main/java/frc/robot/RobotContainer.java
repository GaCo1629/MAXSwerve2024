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
import frc.robot.commands.AutoAmp;
import frc.robot.commands.AutoCollect;
import frc.robot.commands.AutoQuickShoot;
import frc.robot.commands.AutoShoot;
import frc.robot.commands.AutoTurnToHeading;
import frc.robot.commands.WaitForTiltInPosition;
import frc.robot.subsystems.BatonSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.LiftSubsystem;
import frc.robot.subsystems.VisionSubsystem;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;

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
    PS4Controller driverController  = new PS4Controller(OIConstants.kDriverControllerPort);
    Joystick  copilot_1             = new Joystick(OIConstants.kCoPilotController1Port);
    Joystick  copilot_2             = new Joystick(OIConstants.kCoPilotController2Port);

    // The robot's subsystems
    public final DriveSubsystem  robotDrive     = new DriveSubsystem(driverController, copilot_1, copilot_2);
    public final BatonSubsystem  baton          = new BatonSubsystem(driverController, copilot_1, copilot_2);
    public final LiftSubsystem   lift           = new LiftSubsystem(driverController, copilot_1, copilot_2);
    public final VisionSubsystem vision         = new VisionSubsystem();
    public final LEDSubsystem    LEDstrip       = new LEDSubsystem(0);

    private final SendableChooser<Command> autoChooser;

    /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
public RobotContainer() {

        // Register named commands
    NamedCommands.registerCommand("Shoot",          new AutoShoot(baton, robotDrive));
    NamedCommands.registerCommand("QuickShoot",     new AutoQuickShoot(baton));
    NamedCommands.registerCommand("Collect",        new AutoCollect(baton, robotDrive));
    NamedCommands.registerCommand("WaitForTilt",    new WaitForTiltInPosition(baton));
    NamedCommands.registerCommand("Amp",            new AutoAmp(baton, robotDrive));
    NamedCommands.registerCommand("ShooterOff",     baton.quickShootingOffCmd());
    NamedCommands.registerCommand("Shooter3500",    baton.quickShootingOnCmd(0, 3500));    
    
    NamedCommands.registerCommand("TurnTo0",        new AutoTurnToHeading(robotDrive,   0, 1.5));
    NamedCommands.registerCommand("TurnTo45",       new AutoTurnToHeading(robotDrive,  45, 1.5));
    NamedCommands.registerCommand("TurnTo90",       new AutoTurnToHeading(robotDrive,  90, 1.5));
    NamedCommands.registerCommand("TurnTo180",      new AutoTurnToHeading(robotDrive, 180, 1.5));
    NamedCommands.registerCommand("TurnTo270",      new AutoTurnToHeading(robotDrive, -90, 1.5));
    NamedCommands.registerCommand("TurnTo305",       new AutoTurnToHeading(robotDrive, -270, 1.5));
    

    NamedCommands.registerCommand("CollectorOn",    baton.collectCmd());
    NamedCommands.registerCommand("CollectorOff",   baton.stopIntakeCmd());
    NamedCommands.registerCommand("RaiseShooter",   robotDrive.setSpeakerTrackingCmd(true));
    NamedCommands.registerCommand("LowerShooter",   robotDrive.setSpeakerTrackingCmd(false));
    
    // Configure the button bindings
    configureButtonBindings();

    autoChooser = AutoBuilder.buildAutoChooser(); // Default auto will be `Commands.none()`
    SmartDashboard.putData("Auto Chooser", autoChooser);

    // Configure default commands
    robotDrive.setDefaultCommand(robotDrive.driveCmd());

  }

  private void configureButtonBindings() {

    // Pilot Functions
    new JoystickButton(driverController, Button.kR1.value)    
        .onTrue(robotDrive.setTurboModeCmd(true))
        .onFalse(robotDrive.setTurboModeCmd(false));
    
    new JoystickButton(driverController, Button.kL1.value)    
        .onTrue(baton.collectCmd())
        .onTrue(robotDrive.setNoteTrackingCmd(true))
        .onFalse(baton.stopIntakeCmd())
        .onFalse(robotDrive.setNoteTrackingCmd(false));
        
    new JoystickButton(driverController, Button.kR2.value)
        .whileTrue(baton.fireCmd());  // Repeats Automatically

    new JoystickButton(driverController, Button.kL2.value)    
        .onTrue(robotDrive.setSpeakerTrackingCmd(true))
        .onFalse(robotDrive.setSpeakerTrackingCmd(false));

    new JoystickButton(driverController, Button.kTouchpad.value)
        .onTrue(robotDrive.resetHeadingCmd());

    new JoystickButton(driverController, Button.kCross.value)
        .onTrue(baton.stopIntakeCmd());

    // Co-Pilot Functions
    new JoystickButton(copilot_1, Button.kL1.value)    
        .onTrue(baton.collectCmd())
        .onFalse(baton.stopIntakeCmd());

    new JoystickButton(copilot_1, Button.kCross.value)
        .onTrue(baton.ejectCmd())
        .onFalse(baton.stopIntakeCmd());

    new JoystickButton(copilot_1, Button.kTouchpad.value)
        .onTrue(baton.amplifyCmd());

    // Manual shooting controls
    new JoystickButton(copilot_1, Button.kL2.value)
        .onTrue(baton.enableManualShootingCmd(true))
        .onFalse(baton.enableManualShootingCmd(false));

    new JoystickButton(copilot_1, Button.kR2.value)
        .whileTrue(baton.fireCmd());  // Repeats Automatically

    new POVButton(copilot_1, 0)
        .onTrue(baton.bumpTiltCmd(2));

    new POVButton(copilot_1, 180)
        .onTrue(baton.bumpTiltCmd(-2));

    new POVButton(copilot_1, 90)
        .onTrue(baton.bumpShooterCmd(200));

    new POVButton(copilot_1, 270)
        .onTrue(baton.bumpShooterCmd(-200));


    // Add a button to run the auto to SmartDashboard, this will also be in the auto chooser built above
    SmartDashboard.putData("1-3-5",  new PathPlannerAuto("1-3-5"));
    SmartDashboard.putData("1-2",  new PathPlannerAuto("1-2"));
  
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
