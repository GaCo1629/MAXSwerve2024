// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class LEDSubsystem extends SubsystemBase {
  private LEDmode                 lastMode = LEDmode.ALLIANCE;
  private int                     stripLength;
  private AddressableLED          ledStrip;
  private Addressable2815LEDBuffer ledBuffer;  // Use the new class that flips the R&G LEDs

  
  // members for different modes
  private int patternMarker = 0;
  private int direction = 1;
  private int collectingLEDSpeed = 3;

  /** Creates a new LED Strip. */
  public LEDSubsystem(int LEDs, int port) {
    stripLength = LEDs;
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
      clear();
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

      case COLLECTING:  // Seeking a Note to collect
        showCollecting();
        break;

      case NOTE_DETECTED:      // Note is visible
        break;

      case NOTE_COLLECTED:     // Note is in robot
        break;

      case AIMING:             // Seeling a speaker to score
        break;

      case SPEAKER_DETECTED:   // Speaker Apriltag has been detected
        break;

      case SHOOTING:           // In the process of preparing to shoot and waiting to shoot 
        break;

      case SPEEDOMETER:        // Displaying robot speed on power meter.
        break;

      case SYSTEM_ERROR:       // Displaying system error code
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
        ledBuffer.setRGB(patternMarker, 128,0,0);
      } else {
        ledBuffer.setRGB(patternMarker, 0,0,128);
      }
    }
  }

  // -----------------------------------------------------------------------------------
  private void showRainbow() {
    // For every pixel
    for (var i = 0; i < stripLength; i++) {
      // Calculate the hue - hue is easier for rainbows because the color
      // shape is a circle so only one value needs to precess
      final var hue = (patternMarker + (i * 180 / stripLength)) % 180;
      // Set the value
      ledBuffer.setHSV(i, hue, 255, 128);
    }
    // Increase by to make the rainbow "move"
    patternMarker += 3;
    // Check bounds
    patternMarker %= 180;
  }

  // -----------------------------------------------------------------------------------
  private void showCollecting(){
    //Clear the beginning of the light cluster
    for (int i = 0; i < collectingLEDSpeed; i++){
      ledBuffer.setRGB((patternMarker + i) % stripLength, 0, 0, 0);
    }

    //Wrap around at the end of the strand
    if (patternMarker >= (stripLength - 1)) {
      ledBuffer.setRGB(patternMarker, 0, 0, 0);
      patternMarker = 0; 
    }

    //Increment pattern marker and checking bounds
    patternMarker += collectingLEDSpeed;
    patternMarker %= stripLength;
    //Paint light cluster
    for (int i = 0; i < 15; i++){
      ledBuffer.setRGB((patternMarker + i)%stripLength, 200, 20, 0);
    }
  }
    
  // ==========================================================================
  //  Utility methods
  // ==========================================================================
 
  private void clear(){
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

