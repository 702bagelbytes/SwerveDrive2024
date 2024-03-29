// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;

public class ArmPIDCommand extends Command {

  private PIDController ArmPID = new PIDController(
      Constants.ArmConstants.kP,
      Constants.ArmConstants.kI,
      Constants.ArmConstants.kD);

  ArmSubsystem armsubsystem;

  /** Creates a new ArmPIDCommand. */
  public ArmPIDCommand(ArmSubsystem a_ArmSubsystem, double setpoint) {
    this.armsubsystem = a_ArmSubsystem;
    ArmPID.setSetpoint(setpoint);
    ArmPID.setTolerance(Constants.ArmConstants.ArmPIDTolerance);
    addRequirements(a_ArmSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double value = ArmPID.calculate(armsubsystem.getArmAngle());
    armsubsystem.set(value);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    armsubsystem.set(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return ArmPID.atSetpoint();
  }
}
