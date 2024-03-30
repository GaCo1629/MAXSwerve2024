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
import frc.robot.commands.AutoFindNoteLater;
import frc.robot.commands.AutoShoot;
import frc.robot.commands.AutoShootNow;
import frc.robot.commands.AutoTurnToHeading;
import frc.robot.commands.DefaultDriveCommand;
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
    NamedCommands.registerCommand("ShootNow",       new AutoShootNow(baton, 0, 2800));
    NamedCommands.registerCommand("ShootNowSA",     new AutoShootNow(baton, 7, 3400));
    NamedCommands.registerCommand("WaitForTilt",    new WaitForTiltInPosition(baton));
    NamedCommands.registerCommand("FindNote",       new AutoFindNote(vision));
    NamedCommands.registerCommand("FindNoteLater",  new AutoFindNoteLater(vision));
    NamedCommands.registerCommand("LookNow",        Commands.runOnce(() -> Globals.setStartNoteFinding()));
    NamedCommands.registerCommand("SpinUpLong",     Commands.runOnce(() -> baton.setShooterRPM(1500)));
    NamedCommands.registerCommand("SpinUpTrap",     Commands.runOnce(() -> baton.setShooterRPM(2600)));
    NamedCommands.registerCommand("SpinUpShort",    Commands.runOnce(() -> baton.setShooterRPM(1000)));
    NamedCommands.registerCommand("StopShooter",    Commands.runOnce(() -> baton.setShooterRPM(0)));
    NamedCommands.registerCommand("Lob",            Commands.runOnce(() -> baton.lob()));
    NamedCommands.registerCommand("ReadyShooter3s", Commands.runOnce(() -> baton.readyShooter(3.7)));
    NamedCommands.registerCommand("ReadyShooterClose", Commands.runOnce(() -> baton.readyShooter(2.4))); //Range should be calibrated on field
    

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

    NamedCommands.registerCommand("CollectorOn",    Commands.runOnce(() -> baton.collect()));
    NamedCommands.registerCommand("CollectorOff",   Commands.runOnce(() -> baton.stopIntake()));
    
    // Configure the button bindings
    configureButtonBindings();

    autoChooser = AutoBuilder.buildAutoChooser(); // Default auto will be `Commands.none()`
    SmartDashboard.putData("Auto Chooser", autoChooser);

    // Configure default commands
    robotDrive.setDefaultCommand(new DefaultDriveCommand(robotDrive));

    Globals.startingLocationSet = false ;
}


private void configureButtonBindings() {

    // -----------------   Pilot Functions
    // Turbo
    new JoystickButton(driverController, Button.kR1.value)    
        .onTrue(Commands.runOnce(() -> robotDrive.setTurboOn()))
        .onFalse(Commands.runOnce(() -> robotDrive.setTurboOff()));

    // Speaker Aim
    new JoystickButton(driverController, Button.kL2.value)    
        .onTrue(Commands.runOnce(() -> baton.setSpeakerTracking(true)))
        .onFalse(Commands.runOnce(() -> baton.setSpeakerTracking(false)))
        .onFalse(Commands.runOnce(() -> baton.relaxBaton()));

    // Shoot    
    new JoystickButton(driverController, Button.kR2.value)
        .whileTrue(Commands.runOnce(() -> baton.fire()))  // Repeats Automatically
        .onTrue(Commands.runOnce(() -> robotDrive.updateOdometryFromSpeaker()));  

    // Turn to Lob to Amp
    new JoystickButton(driverController, Button.kTriangle.value)    
        .onTrue(Commands.runOnce(() -> robotDrive.turnToFaceAmp()));
    
    // Turn to source
    new JoystickButton(driverController, Button.kSquare.value)    
        .onTrue(Commands.runOnce(() -> robotDrive.turnToSource()));
    
       // Collect
    new JoystickButton(driverController, Button.kL1.value)    
        .onTrue(Commands.runOnce(() -> baton.collect()))
        .onTrue(Commands.runOnce(() -> baton.setNoteTracking(true)))
        .onFalse(Commands.runOnce(() -> baton.stopIntake()))
        .onFalse(Commands.runOnce(() -> baton.setNoteTracking(false)));
   
    // Reset Heading    
    new JoystickButton(driverController, Button.kTouchpad.value)
        .onTrue(Commands.runOnce(() -> robotDrive.resetHeading()));

    //  Eject Note   
    new JoystickButton(driverController, Button.kCross.value)
        .onTrue(Commands.runOnce(() -> baton.eject()))
        .onFalse(Commands.runOnce(() -> baton.stopIntake()));

    // --------------   Co-Pilot Functions

    // Fire
    new JoystickButton(copilot_1, Button.kR2.value)
        .whileTrue(Commands.runOnce(() -> baton.fire()));  // Repeats Automatically

    // Manual Collect
    new JoystickButton(copilot_1, Button.kL1.value)    
        .onTrue(Commands.runOnce(() -> baton.collect()))
        .onFalse(Commands.runOnce(() -> baton.stopIntake()));

    // Eject Note
    new JoystickButton(copilot_1, Button.kCross.value)
        .onTrue(Commands.runOnce(() -> baton.eject()))
        .onFalse(Commands.runOnce(() -> baton.stopIntake()));

    // Amplify    
    new JoystickButton(copilot_1, Button.kTouchpad.value)
        .onTrue(Commands.runOnce(() -> baton.manualAmplify(true)))
        .onFalse(Commands.runOnce(() -> baton.manualAmplify(false)));

    // Manual shooting controls
    new JoystickButton(copilot_1, Button.kL2.value)
        .onTrue(Commands.runOnce(() -> baton.setManualShooting(true)))
        .onFalse(Commands.runOnce(() -> baton.setManualShooting(false)));


    new POVButton(copilot_1, 0)
        .onTrue(Commands.runOnce(() -> baton.bumpTilt(2)));

    new POVButton(copilot_1, 180)
        .onTrue(Commands.runOnce(() -> baton.bumpTilt(-2)));

    new POVButton(copilot_1, 90)
        .onTrue(Commands.runOnce(() -> baton.bumpShooter(200)));

    new POVButton(copilot_1, 270)
        .onTrue(Commands.runOnce(() -> baton.bumpShooter(-200)));

    // Manual Low Lob
    new JoystickButton(copilot_1, Button.kSquare.value)
        .onTrue(Commands.runOnce(() -> baton.setSpeedAndTilt(BatonConstants.lowNoteShareSpeed, BatonConstants.lowNoteShareAngle)));
    
    // Manual High Lob
    new JoystickButton(copilot_1, Button.kTriangle.value)
        .onTrue(Commands.runOnce(() -> baton.setSpeedAndTilt(BatonConstants.highNoteShareSpeed, BatonConstants.highNoteShareAngle)));
  
    // Return to manual default
    new JoystickButton(copilot_1, Button.kCircle.value)
        .onTrue(Commands.runOnce(() -> baton.setSpeedAndTilt(BatonConstants.defaultRPM, BatonConstants.defaultTilt)));
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
