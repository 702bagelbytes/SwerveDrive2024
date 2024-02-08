// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LimitSwitch extends SubsystemBase {
  /** Creates a new LimitSwitch. */
  private DigitalInput ring = new DigitalInput(Constants.LIMIT_SWITCH_INTAKE);

  public LimitSwitch() {
    
  }

   public boolean isRingIn() {
       
        
        return this.ring.get();
    }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
     SmartDashboard.putBoolean("isRingIn", isRingIn());
  }
}
