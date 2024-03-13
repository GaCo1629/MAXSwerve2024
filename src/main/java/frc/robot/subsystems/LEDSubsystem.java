// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.utils.Globals;
import frc.robot.utils.LEDmode;


public class LEDSubsystem extends SubsystemBase {

  private final int               stripLength = 24;
  private final int               speedoGrn   = 14;
  private final int               speedoOrg   = 6;

  private LEDmode                 lastMode = LEDmode.NONE;
  private AddressableLED          ledStrip;
  private Addressable2815LEDBuffer ledBuffer;  // Use the new class that flips the R&G LEDs
  private Timer                   ledTimer = new Timer();

  
  // members for different modes
  private int patternMarker = 0;
  private int direction = 1;
  private int collectingLEDSpeed = 2;
  private boolean stripOn = false;
   
  public static final int RED      = 0;
  public static final int ORANGE   = 5;
  public static final int GREEN   = 60;
  public static final int BLUE   = 120;
  public static final int PURPLE = 140;

  /** Creates a new LED Strip. */
  public LEDSubsystem(int port) {
    ledStrip = new AddressableLED(port);
    
    ledBuffer = new Addressable2815LEDBuffer(stripLength);
    ledStrip.setLength(stripLength);

    // Set the data
    ledStrip.setData(ledBuffer);
    ledStrip.start();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    if (DriverStation.isDisabled()) {
      if (Globals.noteInIntake) {
        Globals.setLEDMode(LEDmode.ALLIANCE);
      } else {
        Globals.setLEDMode(LEDmode.SYSTEM_ERROR);
      }
    }

    SmartDashboard.putString("LED Mode", Globals.getLEDMode().toString());

    if (Globals.getLEDMode() != lastMode) {
      clearStrip();
      ledTimer.restart();
      stripOn = false;
      lastMode = Globals.getLEDMode();
    }

    switch (Globals.getLEDMode()) {
      case NONE:
        clearStrip();
        break;

      case ALLIANCE:    // Display Alliance color
        showAlliance();
        break;

      case RAINBOW:     // Show a pretty Rainbow
        showRainbow();
        break;

      case NOTE_COLLECTING:  // Seeking a Note to collect
        showCollecting();
        break;

      case NOTE_DETECTED:      // Note is visible
        flashStrip(ORANGE, 0.05, 0.05);
        break;

      case NOTE_HOLDING:       // Note is in robot
        flashStrip(ORANGE, 0.25, 0.0);
        break;

      case SEEKING:             // Seeling a speaker to score
        flashStrip(GREEN, 0.25, 0.25);
        break;

      case SPEAKER_DETECTED:   // Speaker Apriltag has been detected
        flashStrip(GREEN, 0.25, 0.0);
        break;

      case SHOOTING:            // Green
        flashStrip(GREEN, 0.1, 0.1);
        break;

      case SHOOTING_TIMEOUT:    // Waiting to shoot Shooting took too long
        flashStrip(BLUE, 0.25, 0.0);
        break;

      case SPEEDOMETER:        // Displaying robot speed on power meter.
      case DEFAULT:
        showSpeedo();
        break;

      case LOWERING:
        flashStrip(RED, 0.2, 0.05);
        break;

      case WAITING:
        flashStrip(PURPLE, 0.2, 0.2);
        break;

     case DONE_WAITING:
        flashStrip(BLUE, 0.25, 0.0);
        break;

       case SYSTEM_ERROR:       // Displaying system error code
        flashStrip(RED, 0.2, 0.2);
        break;

    }

    // Set the LEDs
    ledStrip.setData(ledBuffer);
  }

  // ===================================================================================
  // Show MODE methods.
  // ===================================================================================
  
  // -----------------------------------------------------------------------------------
  private void showAlliance() {
    // turn off the last LED and then move to the next location.  Bounce at ends
    ledBuffer.setRGB(patternMarker, 0,0,0);
    if (patternMarker == 0) {
      direction = 1;
    } else if (patternMarker == (stripLength - 1)) {
      direction = -1;
    }
    patternMarker += direction; // up or down  

    // Set the LED color based on alliance color.  Green if unknown.
    if (DriverStation.getAlliance().isEmpty()) {
      ledBuffer.setRGB(patternMarker, 0,128,0);
    } else {
      if (DriverStation.getAlliance().get() == Alliance.Red) {
        ledBuffer.setRGB(patternMarker, 255,0,0);
      } else {
        ledBuffer.setRGB(patternMarker, 0,0,255);
      }
    }
  }

  // -----------------------------------------------------------------------------------
  private void showRainbow() {
    // Fill the strip with a full color wheel of hues
    for (int i = 0; i < stripLength; i++) {
      int hue = (patternMarker + (i * 180 / stripLength)) % 180;
      ledBuffer.setHSV(i, hue, 255, 128);
    }
    
    // Increase by to make the rainbow "move"
    // patternMarker += 3;
    // Check bounds
    patternMarker %= 180;
  }

  // -----------------------------------------------------------------------------------
  private void showCollecting(){
    clearStrip();

    //Wrap around at the end of the strand
    if (patternMarker >= (stripLength - 1)) {
      ledBuffer.setRGB(patternMarker, 0, 0, 0);
      patternMarker = 0; 
    }

    //Increment pattern marker and checking bounds
    patternMarker = (patternMarker + collectingLEDSpeed) % stripLength;

    //Paint light cluster
    for (int i = 0; i < 6; i++){
      ledBuffer.setRGB((patternMarker + i) % stripLength, 200, 20, 0);
    }
  }
    
  // -----------------------------------------------------------------------------------
  private void showSpeedo(){
    

    // light up based on robot speed.
    // First band green,
    // next band orange,
    // next band red.

    int speedLEDs = (int)((Globals.speed / DriveConstants.kMaxSpeedMetersPerSecond) * stripLength);
    speedLEDs = MathUtil.clamp(speedLEDs, 0, stripLength - 1);

    //Paint light cluster
    clearStrip();
    for (int i = 0; i < speedLEDs; i++){
      if (i >= (speedoGrn + speedoOrg)) {
        ledBuffer.setHSV( i, RED, 255, 128);
      } else if (i >= speedoGrn){
        ledBuffer.setHSV( i, ORANGE, 255, 128);
      } else {
        ledBuffer.setHSV( i, GREEN, 255, 128);
      }
    }
  }
  // ==========================================================================
  //  Utility methods
  // ==========================================================================

  private void flashStrip(int hue, double onTime, double offTime){
    if (stripOn && ledTimer.hasElapsed(onTime) ) {
      ledTimer.restart();
      if (offTime > 0){
        clearStrip();
        stripOn = false;
      }
    } else if (!stripOn && ledTimer.hasElapsed(offTime)){
      setStrip(hue);
      ledTimer.restart();
      stripOn = true;
    }
  }

  private void setStrip(int hue){
    for (var i = 0; i < stripLength; i++) {
      ledBuffer.setHSV(i, hue, 255, 128);
    }    
  }

  private void clearStrip(){
    for (var i = 0; i < stripLength; i++) {
      ledBuffer.setRGB(i, 0, 0, 0);
    }    
  }
}

// ==========================================================================
// Create a sub-class that flips the red/green LEDs for the 12V 2815 LED strip
class Addressable2815LEDBuffer extends AddressableLEDBuffer {

  // constructor that calls the base constructor.
  public Addressable2815LEDBuffer (int stripLength) {
    super(stripLength)  ;
  }

  // Override setRGB() so it flips the red and green LEDs.  
  // Now any base class methods will also get this change
  @Override
  public void setRGB(int index, int r, int g, int b) {
    super.setRGB(index, g, r, b);
  }
}

