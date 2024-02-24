// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {
  private TalonFX ShooterMotor1 = new TalonFX(Constants.ShooterConstants.BMotorID);
  private TalonFX ShooterMotor2 = new TalonFX(Constants.ShooterConstants.TMotorID);

  /** Creates a new IntakeSubsystem. */
  public ShooterSubsystem() {
    ShooterMotor1.setNeutralMode(NeutralModeValue.Brake);
    ShooterMotor2.setNeutralMode(NeutralModeValue.Brake);
    ShooterMotor1.setInverted(Constants.ShooterConstants.BMotorInvert);
    ShooterMotor2.setInverted(Constants.ShooterConstants.TMotorInvert);

  }

  public void set(double value) {
    ShooterMotor1.set(value);
    ShooterMotor2.set(value);
  }

  public Command runCmd(double value) {
    return this.run(() -> this.set(value));
  }

  @Override
  public void periodic() {

  }

  public Command moveCmd(DoubleSupplier input) {
    return this.runEnd(() -> this.set(input.getAsDouble()), () -> this.set(0));

  }
}
