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

  DoubleSupplier ta;
  BooleanSupplier tv;

  /** Creates a new AutoAim. */
  public AutoFollowCommand(DoubleSupplier ta, BooleanSupplier tv) {
    this.ta = ta;
    this.tv = tv;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    AutoFollowPID.setSetpoint(40);
    AutoFollowPID.setTolerance(1);

    double a = ta.getAsDouble();
    boolean Target = tv.getAsBoolean();
    double value = AutoFollowPID.calculate(a);
    double result = value > 0? value + 0.0955: value - 0.0955;
    RobotContainer.FollowPID = Target ? MathUtil.clamp(result, -0.57, 0.57) : 0;
    SmartDashboard.putNumber("FPID", value);
    SmartDashboard.putNumber("FollowPID", RobotContainer.FollowPID);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.FollowPID = 0;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
