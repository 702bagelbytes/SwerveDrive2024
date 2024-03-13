// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimberSubsystem extends SubsystemBase {
  private TalonFX Lmotor = new TalonFX(Constants.ClimberConstants.LeftLiftMotorID);
  private TalonFX Rmotor = new TalonFX(Constants.ClimberConstants.RightLiftMotorID);

  /** Creates a new ClimberSubsystem. */
  public ClimberSubsystem() {
    CurrentLimitsConfigs currentlimits = new CurrentLimitsConfigs()
    .withStatorCurrentLimit(Constants.ClimberConstants.STATOR_CURRENT_LIMIT)
    .withStatorCurrentLimitEnable(Constants.ClimberConstants.ENABLE_STATOR_CURRENT_LIMIT)
    .withSupplyCurrentLimit(Constants.ClimberConstants.CURRENT_LIMIT)
    .withSupplyCurrentLimitEnable(Constants.ClimberConstants.ENABLE_CURRENT_LIMIT)
    .withSupplyCurrentThreshold(Constants.ClimberConstants.CURRENT_THRESHOLD)
    .withSupplyTimeThreshold(Constants.ClimberConstants.CURRENT_THRESHOLD_TIME);

    var Leftlimitconfigs = new SoftwareLimitSwitchConfigs()
        .withForwardSoftLimitEnable(Constants.ClimberConstants.LiftLimitEnable)
        .withForwardSoftLimitThreshold(Constants.ClimberConstants.LeftLiftPosOutValue)
        .withReverseSoftLimitThreshold(Constants.ClimberConstants.LeftLiftPosInValue)
        .withReverseSoftLimitEnable(Constants.ClimberConstants.LiftLimitEnable);

    var Rightlimitconfigs = new SoftwareLimitSwitchConfigs()
        .withForwardSoftLimitEnable(Constants.ClimberConstants.LiftLimitEnable)
        .withForwardSoftLimitThreshold(Constants.ClimberConstants.RightLiftPosOutValue)
        .withReverseSoftLimitThreshold(Constants.ClimberConstants.RightLiftPosInValue)
        .withReverseSoftLimitEnable(Constants.ClimberConstants.LiftLimitEnable);
    

    Lmotor.setInverted(Constants.ClimberConstants.LeftLiftMotorInverted);
    Rmotor.setInverted(Constants.ClimberConstants.RightLiftMotorInverted);
    Lmotor.setNeutralMode(Constants.ClimberConstants.LeftLiftMotorMode);
    Rmotor.setNeutralMode(Constants.ClimberConstants.RightLiftMotorMode);
    Lmotor.getConfigurator().apply(Leftlimitconfigs);
    Rmotor.getConfigurator().apply(Rightlimitconfigs);
    Lmotor.getConfigurator().apply(currentlimits);
    Rmotor.getConfigurator().apply(currentlimits);
  }

  public void setLmotor(double value) {
     Lmotor.set(value);
   // Lmotor.set(valueChecker(value, getLmotorPos(), Constants.ClimberConstants.LeftLiftPosOutValue,
     //   Constants.ClimberConstants.LeftLiftPosInValue));
  }

  public void setRmotor(double value) {
     Rmotor.set(value);
   // Rmotor.set(valueChecker(value, getRmotorPos(), Constants.ClimberConstants.RightLiftPosOutValue,
    //    Constants.ClimberConstants.RightLiftPosInValue));
  }

  public void set(double Lvalue, double Rvalue) {
     Lmotor.set(Lvalue);
     Rmotor.set(Rvalue);
   // Lmotor.set(valueChecker(Lvalue, getLmotorPos(), Constants.ClimberConstants.LeftLiftPosOutValue,
    //    Constants.ClimberConstants.LeftLiftPosInValue));
    //Rmotor.set(valueChecker(Rvalue, getRmotorPos(), Constants.ClimberConstants.RightLiftPosOutValue,
     //   Constants.ClimberConstants.RightLiftPosInValue));
  }

  public double valueChecker(double value, double pos, double OutPos, double InPos) {
    if (value > 0) {
      return (pos < OutPos) ? value : 0;
    } else {
      return (pos > InPos) ? value : 0;
    }
  }

  public double getLmotorPos() {
    return Lmotor.getPosition().getValueAsDouble();
  }

  public double getRmotorPos() {
    return Rmotor.getPosition().getValueAsDouble();
  }

  public Command moveLeftLiftMotorCmd(DoubleSupplier input) {
    return this.runEnd(() -> this.setLmotor(MathUtil.clamp(input.getAsDouble(),
        -Constants.ClimberConstants.MaxLiftSpeed, Constants.ClimberConstants.MaxLiftSpeed)), () -> this.setLmotor(0));

  }

  public Command moveRightLiftMotorCmd(DoubleSupplier input) {
    return this.runEnd(() -> this.setRmotor(MathUtil.clamp(input.getAsDouble(),
        -Constants.ClimberConstants.MaxLiftSpeed, Constants.ClimberConstants.MaxLiftSpeed)), () -> this.setRmotor(0));

  }

  public Command moveCmd(DoubleSupplier Lval, DoubleSupplier Rval) {
    return this.runEnd(() -> this.set(
        MathUtil.applyDeadband(MathUtil.clamp(Lval.getAsDouble(), -Constants.ClimberConstants.MaxLiftSpeed,
            Constants.ClimberConstants.MaxLiftSpeed), 0.1),
        MathUtil.applyDeadband(MathUtil.clamp(Rval.getAsDouble(), -Constants.ClimberConstants.MaxLiftSpeed,
            Constants.ClimberConstants.MaxLiftSpeed), 0.1)),
        () -> this.set(0, 0));

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("LeftLiftMotorPos", getLmotorPos());
    SmartDashboard.putNumber("RightLiftMotorPos", getRmotorPos());
  }
}
