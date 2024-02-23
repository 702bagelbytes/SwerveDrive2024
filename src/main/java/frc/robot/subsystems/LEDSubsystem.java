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

  public LEDSubsystem() {
    led.setLength(Constants.LEDConstants.LEDLength);
    led.setData(buffer);
    led.start();
  }
  
   public void setColor(Color color) {
    for(int i = 0; i < buffer.getLength(); ++i) {
        buffer.setLED(i, color);
    }
   }
/* 
   public void rainbow(){
    for(int i = 0; i < buffer.getLength(); i++){
      double hue = (m_rainbowFirstPixelHue + (i*));
    }
   }
*/
  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  }
}
