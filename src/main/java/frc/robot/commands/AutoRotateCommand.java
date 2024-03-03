// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class AutoRotateCommand extends Command {
  boolean interrupted;
  double angle;
  Swerve s_Swerve;

  private PIDController AutoAimPID = new PIDController(
      Constants.AutoAimConstants.kP,
      Constants.AutoAimConstants.kI,
      Constants.AutoAimConstants.kD);

  
  /** Creates a new AutoAim. */
  public AutoRotateCommand(double angle, Swerve s_Swerve) {
    this.angle = angle;
    

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    AutoAimPID.setSetpoint(angle);
    AutoAimPID.setTolerance(Constants.AutoAimConstants.AutoAimPIDTolerance);

    double x = s_Swerve.getGyroYaw();
    
    double value = AutoAimPID.calculate(x);
    double result = Math.copySign(Math.abs(value) + 0.0955, value); 
    double AimPID = (Target ? MathUtil.clamp(result, -0.57, 0.57) : 0);

    s_Swerve.drive(
                new Translation2d(FollowPID, 0).times(Constants.Swerve.MAX_SPEED),
                AimPID * Constants.Swerve.MAX_ANGULAR_VELOCITY,
                !false,
                false);
   

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.setAimPID(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return AutoAimPID.atSetpoint();
  }
}
