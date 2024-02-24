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
  private LEDmode              mode = LEDmode.RAINBOW;
  private int                  stripLength;
  private AddressableLED       led;
  private AddressableLEDBuffer ledBuffer;

  // members for different modes
  private int patternMarker = 0;
  private int direction = 1;

  /** Creates a new LED. */
  public LEDSubsystem(int LEDs, int port) {
    stripLength = LEDs;
    led = new AddressableLED(port);
    
    ledBuffer = new AddressableLEDBuffer(stripLength);
    led.setLength(stripLength);

    // Set the data
    led.setData(ledBuffer);
    led.start();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    switch (mode) {
      case ALLIANCE:
        showAlliance();
        break;

      case RAINBOW:
        showRainbow();
        break;
    }

    // Set the LEDs
    led.setData(ledBuffer);
  }

  // ===================================================================================
  // Show MODE methods.
  // ===================================================================================
  
  private void showAlliance() {
    // turn off the last LED and then move to thenext location.  Bounce at ends
    ledBuffer.setRGB(patternMarker, 0,0,0);
    if (patternMarker == 0) {
      direction = 1;
    } else if (patternMarker == (stripLength - 1)) {
      direction = -1;
    }

    // Set the LED color based on alliance color
    patternMarker += direction; // up or down  
    if (DriverStation.getAlliance().get() == Alliance.Red) {
      ledBuffer.setRGB(patternMarker, 128,0,0);
    } else {
      ledBuffer.setRGB(patternMarker, 0,0,128);
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
    
  //  Utility methods
  public void setMode(LEDmode mode) {
    clear();
    this.mode = mode;
  }

  private void clear(){
    for (var i = 0; i < stripLength; i++) {
      ledBuffer.setRGB(i, 0, 0, 0);
    }    
    led.setData(ledBuffer);
  }
}
