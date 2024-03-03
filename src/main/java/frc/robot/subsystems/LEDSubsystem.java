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
  private AddressableLED led1 = new AddressableLED(Constants.LEDConstants.LED_1_PwmID);
  private AddressableLEDBuffer buffer1 = new AddressableLEDBuffer(Constants.LEDConstants.LED_1_Length);
  private int m_rainbowFirstPixelHue = 0;
  private boolean do_the_rainbow = true;

  public LEDSubsystem() {
    led1.setLength(Constants.LEDConstants.LED_1_Length);
    led1.setData(buffer1);
    led1.start();
    rainbow();
  }
  
   public void setColor(Color color) {
    do_the_rainbow = false;
    for(int i = 0; i < buffer1.getLength(); ++i) {
        buffer1.setLED(i, color);
    }
    led1.setData(buffer1);}

    public void DoTheRainbow(boolean val) {
    do_the_rainbow = val;
    
   }

   public void bagelbow() {
    // NDB: TODO: Get the colors right and then create a private
    // variable for the first index that can be looped over
    buffer1.setHSV(0, 55, 40, 128);  
    buffer1.setHSV(1, 56, 100, 128);
    buffer1.setHSV(2, 57, 160, 128);
    buffer1.setHSV(3, 58, 160, 128);
    buffer1.setHSV(4, 59, 160, 128);
    buffer1.setHSV(5, 60, 100, 128);
    buffer1.setHSV(6, 61, 40, 128);

    buffer1.setHSV(7, 175, 40, 128);
    buffer1.setHSV(8, 176, 100, 128);
    buffer1.setHSV(9, 177, 160, 128);
    buffer1.setHSV(10, 178, 160, 128);
    buffer1.setHSV(11, 179, 160, 128);
    buffer1.setHSV(12, 180, 160, 128);
    buffer1.setHSV(13, 181, 160, 128);

    led1.setData(buffer1);
   }
 
   public void rainbow(){
    // For every pixel
    for (var i = 0; i < 14; i++) {
      // Calculate the hue - hue is easier for rainbows because the color
      // shape is a circle so only one value needs to precess
      final var hue = (m_rainbowFirstPixelHue + (i * 180 / 14)) % 180;
      // Set the value
      buffer1.setHSV(i, hue, 255, 128);
      buffer1.setHSV(i+14, hue, 255, 128);
    }
    led1.setData(buffer1);
    // Increase by to make the rainbow "move"
    m_rainbowFirstPixelHue += 3;
    // Check bounds
    m_rainbowFirstPixelHue %= 180;
   }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (do_the_rainbow) {
      rainbow();
    }
  }
}
