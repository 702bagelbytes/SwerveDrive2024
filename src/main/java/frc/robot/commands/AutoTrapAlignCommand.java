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

public class AutoTrapAlignCommand extends Command {
  boolean interrupted;

  private PIDController AutoTranslatePID = new PIDController(

      Constants.AutoTranslateConstants.kP,
      Constants.AutoTranslateConstants.kI,
      Constants.AutoTranslateConstants.kD);

  private PIDController AutoRotatePID = new PIDController(
      Constants.AutoRotateConstants.kP,
      Constants.AutoRotateConstants.kI,
      Constants.AutoAimConstants.kD);
  
  private PIDController AutoStrafePID = new PIDController(
      Constants.AutoStrafeConstants.kP,
      Constants.AutoStrafeConstants.kI,
      Constants.AutoStrafeConstants.kD);
    
  
  
  Swerve s_Swerve;
  LimelightBackSubsystem l_LimelightBackSubsystem;

  /** Creates a new AutoAim. */
  public AutoTrapAlignCommand(LimelightBackSubsystem l_LimelightBackSubsystem, Swerve s_Swerve) {
    
    this.s_Swerve = s_Swerve;
    this.LimelightBackSubsystem = l_LimelightBackSubsystem;
    
    addRequirements(s_Swerve);
    addRequirements(l_LimelightBackSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.robotCentric = true;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    AutoTranslatePID.setSetpoint(Constants.AutoTranslateConstants.Setpoint);
    AutoTranslatePID.setTolerance(Constants.AutoTranslateConstants.Tolerance);
    AutoRotatePID.setSetpoint(0);
    AutoRotatePID.setTolerance(Constants.AutoRotateConstants.Tolerance);
    AutoStrafePID.setSetpoint(0);
    AutoStrafePID.setTolerance(Constants.AutoRotateConstants.Tolerance);
    


    double d = l_LimelightBackSubsystem.getTargetDistance();
    boolean Target = l_LimelightBackSubsystem.IsTargetAvailable();
    double value = TranslatePID.calculate(d);
    double result = value > 0 ? value + 0.0955 : value - 0.0955;
    double TranslatePID = (Target ? MathUtil.clamp(result, -0.20, 0.20) : 0);

    double x = l_LimelightBackSubsystem.getTargetX();
    double value = TranslatePID.calculate(x);
    double result = value > 0 ? value + 0.0955 : value - 0.0955;
    double StrafePID = (Target ? MathUtil.clamp(result, -0.20, 0.20) : 0);

    double tYaw = l_LimelightBackSubsystem.getTargetPos(5);
    double value2 = AutoRotatePID.calculate(tYaw);
    double result2 = Math.copySign(Math.abs(value2) + 0.0955, value2); 
    double RotatePID = (Target ? MathUtil.clamp(result2, -0.47, 0.47) : turn);
    
   
    s_Swerve.drive(
                new Translation2d(TranslatePID, StrafePID).times(Constants.Swerve.MAX_SPEED),
                RotatePID * Constants.Swerve.MAX_ANGULAR_VELOCITY,
                !true,
                false);

    SmartDashboard.putNumber("RPID", value2);
    SmartDashboard.putNumber("tYaw", tYaw);
    SmartDashboard.putNumber("TPID", value);
    SmartDashboard.putNumber("tD", d);
    SmartDashboard.putNumber("SPID", value3);
    SmartDashboard.putNumber("tx", x);
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
    return (TranslatePID.atSetpoint() && StrafePID.atSetpoint() && RotatePID.atSetpoint())? true : false;
  }
}
