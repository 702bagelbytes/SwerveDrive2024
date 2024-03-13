// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
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
  CurrentLimitsConfigs currentlimits = new CurrentLimitsConfigs()
    .withStatorCurrentLimit(Constants.ShootConstants.STATOR_CURRENT_LIMIT)
    .withStatorCurrentLimitEnable(Constants.ShootConstants.ENABLE_STATOR_CURRENT_LIMIT)
    .withSupplyCurrentLimit(Constants.ShootConstants.CURRENT_LIMIT)
    .withSupplyCurrentLimitEnable(Constants.ShootConstants.ENABLE_CURRENT_LIMIT)
    .withSupplyCurrentThreshold(Constants.ShootConstants.CURRENT_THRESHOLD)
    .withSupplyTimeThreshold(Constants.ShootConstants.CURRENT_THRESHOLD_TIME);

    ShooterMotor1.setNeutralMode(Constants.ShootConstants.TopShootMotorMode);
    ShooterMotor2.setNeutralMode(Constants.ShootConstants.BottomShootMotorMode);
    ShooterMotor1.setInverted(Constants.ShootConstants.TopShootMotorInverted);
    ShooterMotor2.setInverted(Constants.ShootConstants.BottomShootMotorInverted);
    // var talonFXConfigs = new TalonFXConfiguration();

    // // set slot 0 gains
    // var slot0Configs = talonFXConfigs.Slot0;
    // slot0Configs.kS = 0.25; // Add 0.25 V output to overcome static friction
    // slot0Configs.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
    // slot0Configs.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
    // slot0Configs.kP = 0.11; // An error of 1 rps results in 0.11 V output
    // slot0Configs.kI = 0; // no output for integrated error
    // slot0Configs.kD = 0; // no output for error derivative

    // // set Motion Magic Velocity settings
    // var motionMagicConfigs = talonFXConfigs.MotionMagic;
    // motionMagicConfigs.MotionMagicAcceleration = 400; // Target acceleration of 400 rps/s (0.25 seconds to max)
    // motionMagicConfigs.MotionMagicJerk = 4000; // Target jerk of 4000 rps/s/s (0.1 seconds)

    // ShooterMotor1.getConfigurator().apply(talonFXConfigs);
    // ShooterMotor2.getConfigurator().apply(talonFXConfigs);
    ShooterMotor1.getConfigurator().apply(currentlimits);
    ShooterMotor2.getConfigurator().apply(currentlimits);
  }

  final MotionMagicVelocityVoltage m_request = new MotionMagicVelocityVoltage(0);

  public void set(double value) {
    // ShooterMotor1.setControl(m_request.withVelocity(value));
    // ShooterMotor2.setControl(m_request.withVelocity(value));

    value = MathUtil.applyDeadband(value, 0.1);
    ShooterMotor1.setVoltage(value * 12);
    ShooterMotor2.setVoltage(value * 12);
  }

  public void set(double top, double bottom) {
    // ShooterMotor1.setControl(m_request.withVelocity(top));
    // ShooterMotor2.setControl(m_request.withVelocity(bottom));


    top = MathUtil.applyDeadband(top, 0.1);
    bottom = MathUtil.applyDeadband(bottom, 0.1);

    ShooterMotor1.setVoltage(top * 12);
    ShooterMotor2.setVoltage(bottom * 12);
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
