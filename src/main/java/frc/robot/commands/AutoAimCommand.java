// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class AutoAimCommand extends Command {
  boolean interrupted;

  private PIDController AutoAimPID = new PIDController(

      Constants.AutoAimConstants.kP,
      Constants.AutoAimConstants.kI,
      Constants.AutoAimConstants.kD);
  DoubleSupplier tx;
  BooleanSupplier tv;

  /** Creates a new AutoAim. */
  public AutoAimCommand(DoubleSupplier tx, BooleanSupplier tv) {
    this.tv = tv;
    this.tx = tx;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    AutoAimPID.setSetpoint(0);
    AutoAimPID.setTolerance(1);

    double x = tx.getAsDouble();
    boolean Target = tv.getAsBoolean();
    double value = AutoAimPID.calculate(x);
    double result = value > 0? value + 0.0955: value - 0.0955;
    RobotContainer.AimPID = Target ? MathUtil.clamp(result, -0.57, 0.57) : 0;
    SmartDashboard.putNumber("APID", value);
    SmartDashboard.putNumber("ran", 1);
    SmartDashboard.putNumber("Atx", x);
    SmartDashboard.putNumber("AimPID", RobotContainer.AimPID);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.AimPID = 0;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
