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

public class AutoFollowCommand extends Command {
  boolean interrupted;

  private PIDController AutoFollowPID = new PIDController(

      Constants.AutoFollowConstants.kP,
      Constants.AutoFollowConstants.kI,
      Constants.AutoFollowConstants.kD);

      private PIDController AutoAimPID = new PIDController(
        Constants.AutoAimConstants.kP,
        Constants.AutoAimConstants.kI,
        Constants.AutoAimConstants.kD);
  
  
  DoubleSupplier tx;
  DoubleSupplier ta;
  BooleanSupplier tv;

  /** Creates a new AutoAim. */
  public AutoFollowCommand(DoubleSupplier tx, DoubleSupplier ta, BooleanSupplier tv) {
    this.ta = ta;
    this.tx = tx;
    this.tv = tv;
    

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    AutoFollowPID.setSetpoint(50);
    AutoFollowPID.setTolerance(1);
    AutoAimPID.setSetpoint(0);
    AutoAimPID.setTolerance(1);


    double a = ta.getAsDouble();
    boolean Target = tv.getAsBoolean();
    double value = AutoFollowPID.calculate(a);
    double result = value > 0 ? value + 0.0955 : value - 0.0955;
    RobotContainer.setFollowPID(Target ? MathUtil.clamp(result, -0.67, 0.67) : 0);
    SmartDashboard.putNumber("FPID", value);

    double x = tx.getAsDouble();

    double value2 = AutoAimPID.calculate(x);
    double result2 = Math.copySign(Math.abs(value2) + 0.0955, value2); 
    // value > 0 ? value + 0.0955 : value - 0.0955;
    RobotContainer.setAimPID(Target ? MathUtil.clamp(result2, -0.57, 0.57) : 0);
    // SmartDashboard.putNumber("FollowPID", RobotContainer.FollowPID);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.setFollowPID(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
