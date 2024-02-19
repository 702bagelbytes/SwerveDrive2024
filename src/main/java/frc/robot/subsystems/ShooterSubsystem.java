// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.hardware.TalonFX;


import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {
  private TalonFX ShooterMotor1 = new TalonFX(Constants.ShootConstants.TopShootMotorID);
  private TalonFX ShooterMotor2 = new TalonFX(Constants.ShootConstants.BottomShootMotorID);

  /** Creates a new IntakeSubsystem. */
  public ShooterSubsystem() {
    ShooterMotor1.setNeutralMode(Constants.ShootConstants.TopShootMotorMode);
    ShooterMotor2.setNeutralMode(Constants.ShootConstants.BottomShootMotorMode);
    ShooterMotor1.setInverted(Constants.ShootConstants.TopShootMotorInverted);
    ShooterMotor2.setInverted(Constants.ShootConstants.BottomShootMotorInverted);
    

  }

  public void set(double value) {
    value = MathUtil.applyDeadband(value, 0.1);
    ShooterMotor1.set(value);
    ShooterMotor2.set(value);
  }

  public void set(double top, double bottom) {
    top = MathUtil.applyDeadband(top, 0.1);
    bottom = MathUtil.applyDeadband(bottom, 0.1);

    ShooterMotor1.set(top);
    ShooterMotor2.set(bottom);
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
