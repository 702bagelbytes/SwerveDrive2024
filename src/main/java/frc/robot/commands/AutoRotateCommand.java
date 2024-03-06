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

public class AutoRotateCommand extends Command {
  boolean interrupted;

 

      private PIDController AutoAimPID = new PIDController(
        Constants.AutoAimConstants.kP,
        Constants.AutoAimConstants.kI,
        Constants.AutoAimConstants.kD);
  
  
  double angle;
  Swerve s_Swerve;
  double translation;
  double strafe;

  /** Creates a new AutoAim. */
  public AutoRotateCommand(double angle, Swerve s_Swerve, double translation, double strafe) {
    this.angle = angle;
    this.s_Swerve = s_Swerve;
    this.translation = translation;
    this.strafe = strafe;
    
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
    
    AutoAimPID.setSetpoint(angle);
    AutoAimPID.setTolerance(1);
    


    

    double a = s_Swerve.getGyroYaw().getDegrees();

    double value2 = AutoAimPID.calculate(a);
    double result2 = Math.copySign(Math.abs(value2) + 0.0955, value2); 
    // value > 0 ? value + 0.0955 : value - 0.0955;
    double AimPID =  MathUtil.clamp(result2, -0.57, 0.57);
    // SmartDashboard.putNumber("FollowPID", RobotContainer.FollowPID);

    s_Swerve.drive(
                new Translation2d(translation, strafe).times(Constants.Swerve.MAX_SPEED),
                AimPID * Constants.Swerve.MAX_ANGULAR_VELOCITY,
                !false,
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
    return false;
  }
}
