// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {
  private TalonFX ShooterMotor1 = new TalonFX(17);
  private TalonFX ShooterMotor2 = new TalonFX(16);

  /** Creates a new IntakeSubsystem. */
  public ShooterSubsystem() {
    ShooterMotor1.setNeutralMode(NeutralModeValue.Brake);
    ShooterMotor2.setNeutralMode(NeutralModeValue.Brake);
    ShooterMotor1.setInverted(false);
    ShooterMotor2.setInverted(true);
    

  }

  public void set(double value) {
    value = MathUtil.applyDeadband(value, 0.1);
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
