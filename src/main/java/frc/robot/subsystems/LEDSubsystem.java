// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;



public class LEDSubsystem extends SubsystemBase {

  private final int               stripLength = 24;
  private final int               speedoGrn   = 14;
  private final int               speedoOrg   = 6;

  private LEDmode                 lastMode = LEDmode.ALLIANCE;
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

    if (Globals.ledMode != lastMode) {
      clearStrip();
      ledTimer.restart();
      stripOn = false;
      lastMode = Globals.ledMode;
    }

    switch (Globals.ledMode) {
      case ALLIANCE:    // Display Alliance color
      case DEFAULT:
        showAlliance();
        break;

      case RAINBOW:     // Show a pretty Rainbow
        showRainbow();
        break;

      case NOTE_COLLECTING:  // Seeking a Note to collect
        showCollecting();
        break;

      case NOTE_DETECTED:      // Note is visible
        flashStrip(ORANGE, 0.1, 0.1);
        break;

      case NOTE_HOLDING:       // Note is in robot
        flashStrip(ORANGE, 0.25, 0.0);
        break;

      case SEEKING:             // Seeling a speaker to score
        flashStrip(GREEN, 0.3, 0.3);
        break;

      case SPEAKER_DETECTED:   // Speaker Apriltag has been detected
        flashStrip(GREEN, 0.25, 0.0);
        break;

      case SHOOTING:           // In the process of preparing to shoot and waiting to shoot 
        flashStrip(GREEN, 0.1, 0.1);
        break;

      case SPEEDOMETER:        // Displaying robot speed on power meter.
        showSpeedo();
        break;

      case SYSTEM_ERROR:       // Displaying system error code
        flashStrip(RED, 0.1, 0.1);
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
    clearStrip();

    // light up based on robot speed.
    // First band green,
    // next band orange,
    // next band red.

    int speedLEDs = (int)(Globals.speed * stripLength);

    //Paint light cluster
    for (int i = 0; i < stripLength; i++){
      if (speedLEDs > (speedoGrn + speedoOrg)) {
        ledBuffer.setHSV( i, RED, 255, 128);
      } else if (speedLEDs > speedoGrn){
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
    if (stripOn && ledTimer.hasElapsed(onTime)) {
      clearStrip();
      ledTimer.restart();
      stripOn = false;
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

