// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.HardwareLimitSwitchConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ForwardLimitSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class IntakeSubsystem extends SubsystemBase {
  private TalonFX IntakeMotor = new TalonFX(Constants.IntakeConstants.IntakeMotorID);
  
 

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
    var limitConfig = new HardwareLimitSwitchConfigs();
    limitConfig.ForwardLimitSource = ForwardLimitSourceValue.LimitSwitchPin;
    limitConfig.ForwardLimitRemoteSensorID = Constants.LIMIT_SWITCH_INTAKE;
    limitConfig.ForwardLimitEnable = true;
    
    
    IntakeMotor.setNeutralMode(NeutralModeValue.Brake);
    IntakeMotor.getConfigurator().apply(limitConfig);

    
  }

  public void set(double value) {
    IntakeMotor.setVoltage(value * 12);
  }
  public void setMode(NeutralModeValue mode) {
    IntakeMotor.setNeutralMode(mode);
  }

  public Command runCmd(double value) {
    return this.run(() -> this.set(value));
  }
  //public Command runwhileCmd(double value, boolean run) {
    
    
 // }

  @Override
  public void periodic() {

  }

  public Command moveCmd(DoubleSupplier input) {
    return this.runEnd(() -> this.set(input.getAsDouble()), () -> this.set(0));

  }
  public Command runEndCmd(double input) {
    return this.runEnd(() -> this.set(input), () -> this.set(0));

  }
}
