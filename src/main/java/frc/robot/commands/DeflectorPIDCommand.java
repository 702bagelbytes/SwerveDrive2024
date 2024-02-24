// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.Constants;

import frc.robot.subsystems.DeflectorSubsystem;

public class DeflectorPIDCommand extends Command {

  private PIDController DeflectorPID = new PIDController(
      Constants.DeflectorConstants.kP,
      Constants.DeflectorConstants.kI,
      Constants.DeflectorConstants.kD);

  DeflectorSubsystem d_DeflectorSubsystem;

  /** Creates a new ArmPIDCommand. */
  public DeflectorPIDCommand(DeflectorSubsystem d_DeflectorSubsystem, double setpoint) {
    this.d_DeflectorSubsystem = d_DeflectorSubsystem;
    DeflectorPID.setSetpoint(setpoint);
    DeflectorPID.setTolerance(Constants.DeflectorConstants.DeflectorPIDTolerance);
    addRequirements(d_DeflectorSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double value = DeflectorPID.calculate(d_DeflectorSubsystem.getArmAngle());
    d_DeflectorSubsystem.set(value);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    d_DeflectorSubsystem.set(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return DeflectorPID.atSetpoint();
  }
}
