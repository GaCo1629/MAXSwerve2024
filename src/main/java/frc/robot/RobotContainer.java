// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import frc.robot.Constants.BatonConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.AutoAmp;
import frc.robot.commands.AutoCollect;
import frc.robot.commands.AutoFindNote;
import frc.robot.commands.AutoShoot;
import frc.robot.commands.AutoShootNow;
import frc.robot.commands.AutoTurnToHeading;
import frc.robot.commands.WaitForTiltInPosition;
import frc.robot.subsystems.BatonSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.LiftSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.utils.Globals;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

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
    NamedCommands.registerCommand("Amp",            new AutoAmp(baton, robotDrive));
    NamedCommands.registerCommand("Collect",        new AutoCollect(baton, robotDrive));
    NamedCommands.registerCommand("Shoot",          new AutoShoot(baton, robotDrive));
    NamedCommands.registerCommand("ShootNow",       new AutoShootNow(baton, robotDrive, 0, 3000));
    NamedCommands.registerCommand("WaitForTilt",    new WaitForTiltInPosition(baton));
    NamedCommands.registerCommand("FindNote",       new AutoFindNote(vision, 0 , false));
    NamedCommands.registerCommand("FindNoteLater",  new AutoFindNote(vision, 0 , true));
    NamedCommands.registerCommand("LookNow",        Commands.runOnce(() -> Globals.setStartNoteFinding()));

    NamedCommands.registerCommand("TurnTo0",        new AutoTurnToHeading(robotDrive, 0, 2.0));
    NamedCommands.registerCommand("TurnTo20",       new AutoTurnToHeading(robotDrive, 20, 2.0));
    NamedCommands.registerCommand("TurnTo45",       new AutoTurnToHeading(robotDrive, 45, 2.0));
    NamedCommands.registerCommand("TurnTo80",       new AutoTurnToHeading(robotDrive, 80, 2.0));
    NamedCommands.registerCommand("TurnTo90",       new AutoTurnToHeading(robotDrive, 90, 2.0));
    NamedCommands.registerCommand("TurnTo135",      new AutoTurnToHeading(robotDrive, 135, 2.0));
    NamedCommands.registerCommand("TurnTo180",      new AutoTurnToHeading(robotDrive, 180, 2.0));
    NamedCommands.registerCommand("TurnTo-20",      new AutoTurnToHeading(robotDrive, -20, 2.0));
    NamedCommands.registerCommand("TurnTo-45",      new AutoTurnToHeading(robotDrive, -45, 2.0));
    NamedCommands.registerCommand("TurnTo-90",      new AutoTurnToHeading(robotDrive, -90, 2.0));

    NamedCommands.registerCommand("CollectorOn",    baton.collectCmd());
    NamedCommands.registerCommand("CollectorOff",   baton.stopIntakeCmd());
    
    // Configure the button bindings
    configureButtonBindings();

    autoChooser = AutoBuilder.buildAutoChooser(); // Default auto will be `Commands.none()`
    SmartDashboard.putData("Auto Chooser", autoChooser);

    // Configure default commands
    robotDrive.setDefaultCommand(robotDrive.driveCmd());

    Globals.startingLocationSet = false ;

  }

  private void configureButtonBindings() {

    // -----------------   Pilot Functions
    // Turbo
    new JoystickButton(driverController, Button.kR1.value)    
        .onTrue(robotDrive.setTurboModeCmd(true))
        .onFalse(robotDrive.setTurboModeCmd(false));
    
    // Collect
    new JoystickButton(driverController, Button.kL1.value)    
        .onTrue(baton.collectCmd())
        .onTrue(baton.setNoteTrackingCmd(true))
        .onFalse(baton.stopIntakeCmd())
        .onFalse(baton.setNoteTrackingCmd(false));

    // Shoot    
    new JoystickButton(driverController, Button.kR2.value)
        .whileTrue(baton.fireCmd())  // Repeats Automatically
        .onTrue(robotDrive.updateOdometryFromSpeakerCmd());  //  test this to see if it works.

    // Speaker Aim
    new JoystickButton(driverController, Button.kL2.value)    
        .onTrue(baton.setSpeakerTrackingCmd(true))
        .onFalse(baton.setSpeakerTrackingCmd(false));

    // Reset Heading    
    new JoystickButton(driverController, Button.kTouchpad.value)
        .onTrue(robotDrive.resetHeadingCmd());

    //  Eject Note   
    new JoystickButton(driverController, Button.kCross.value)
        .onTrue(baton.ejectCmd())
        .onFalse(baton.stopIntakeCmd());

    // --------------   Co-Pilot Functions

    // Manual Collect
    new JoystickButton(copilot_1, Button.kL1.value)    
        .onTrue(baton.collectCmd())
        .onFalse(baton.stopIntakeCmd());

    // Eject Note
    new JoystickButton(copilot_1, Button.kCross.value)
        .onTrue(baton.ejectCmd())
        .onFalse(baton.stopIntakeCmd());

    // Amplify    
    new JoystickButton(copilot_1, Button.kTouchpad.value)
        .onTrue(baton.amplifyCmd(true))
        .onFalse(baton.amplifyCmd(false));


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
        .onTrue(baton.bumpShooterCmd(250));

    new POVButton(copilot_1, 270)
        .onTrue(baton.bumpShooterCmd(-250));

    new JoystickButton(copilot_1, Button.kSquare.value)
        .onTrue(baton.setManualSpeedAndTiltCmd(BatonConstants.lowNoteShareSpeed, BatonConstants.lowNoteShareAngle));
    
    new JoystickButton(copilot_1, Button.kTriangle.value)
        .onTrue(baton.setManualSpeedAndTiltCmd(BatonConstants.highNoteShareSpeed, BatonConstants.highNoteShareAngle));
  
    new JoystickButton(copilot_1, Button.kCircle.value)
        .onTrue(baton.setManualSpeedAndTiltCmd(BatonConstants.defaultRPM, BatonConstants.defaultTilt));
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
