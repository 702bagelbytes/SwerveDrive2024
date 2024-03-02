// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Swerve;

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
  Swerve s_Swerve;

  /** Creates a new AutoAim. */
  public AutoFollowCommand(DoubleSupplier tx, DoubleSupplier ta, BooleanSupplier tv, Swerve s_Swerve) {
    this.ta = ta;
    this.tx = tx;
    this.tv = tv;
    this.s_Swerve = s_Swerve;
    
    addRequirements(s_Swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.robotCentric = true;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    AutoFollowPID.setSetpoint(20);
    AutoFollowPID.setTolerance(1);
    AutoAimPID.setSetpoint(0);
    AutoAimPID.setTolerance(1);


    double a = ta.getAsDouble();
    boolean Target = tv.getAsBoolean();
    double value = AutoFollowPID.calculate(a);
    double result = value > 0 ? value + 0.0955 : value - 0.0955;
    double FollowPID = (Target ? MathUtil.clamp(result, -0.67, 0.67) : 0);
    SmartDashboard.putNumber("FPID", value);
    SmartDashboard.putNumber("Fta", a);

    double x = tx.getAsDouble();

    double value2 = AutoAimPID.calculate(x);
    double result2 = Math.copySign(Math.abs(value2) + 0.0955, value2); 
    // value > 0 ? value + 0.0955 : value - 0.0955;
    double AimPID = (Target ? MathUtil.clamp(result2, -0.57, 0.57) : 0);
    // SmartDashboard.putNumber("FollowPID", RobotContainer.FollowPID);
    SmartDashboard.putNumber("Ftx", x);
    s_Swerve.drive(
                new Translation2d(FollowPID, 0).times(Constants.Swerve.MAX_SPEED),
                AimPID * Constants.Swerve.MAX_ANGULAR_VELOCITY,
                !true,
                false);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
   RobotContainer.FollowPID = 0;
   RobotContainer.AimPID = 0;
   RobotContainer.robotCentric = false;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return RobotContainer.isRingIn;
  }
}
