// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.LimelightSubsystem;

public class AutoAimCommand extends Command {
  boolean interrupted;

  private PIDController AutoAimPID = new PIDController(

      Constants.AutoAimConstants.kP,
      Constants.AutoAimConstants.kI,
      Constants.AutoAimConstants.kD);

  LimelightSubsystem l_LimelightSubsystem;
  PIDSubsystem p_PidSubsystem;

  /** Creates a new AutoAim. */
  public AutoAimCommand(Subsystem l_LimelightSubsystem) {
    this.l_LimelightSubsystem = (LimelightSubsystem) l_LimelightSubsystem;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(l_LimelightSubsystem);

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

    double x = l_LimelightSubsystem.getTargetX();
    boolean Target = l_LimelightSubsystem.IsTargetAvailable();
    double value = AutoAimPID.calculate(x);
    RobotContainer.AimPID = Target ? MathUtil.clamp(value, -0.62, 0.62) : 0;

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
