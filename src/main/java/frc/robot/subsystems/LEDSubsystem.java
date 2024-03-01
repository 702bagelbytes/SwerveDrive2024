// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LEDSubsystem extends SubsystemBase {
  /** Creates a new LimitSwitch. */
  private AddressableLED led = new AddressableLED(Constants.LEDConstants.PwmID);
  private AddressableLEDBuffer buffer = new AddressableLEDBuffer(Constants.LEDConstants.LEDLength);
  private int m_rainbowFirstPixelHue = 0;

  public LEDSubsystem() {
    led.setLength(Constants.LEDConstants.LEDLength);
    led.setData(buffer);
    led.start();
    rainbow();
  }
  
   public void setColor(Color color) {
    for(int i = 0; i < buffer.getLength(); ++i) {
        buffer.setLED(i, color);
    }
   }
 
   public void rainbow(){
    // For every pixel
    for (var i = 0; i < buffer.getLength(); i++) {
      // Calculate the hue - hue is easier for rainbows because the color
      // shape is a circle so only one value needs to precess
      final var hue = (m_rainbowFirstPixelHue + (i * 180 / buffer.getLength())) % 180;
      // Set the value
      buffer.setHSV(i, hue, 255, 128);
    }
    // Increase by to make the rainbow "move"
    m_rainbowFirstPixelHue += 3;
    // Check bounds
    m_rainbowFirstPixelHue %= 180;
   }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  }
}
