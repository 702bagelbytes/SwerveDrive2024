// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;


public class DeflectorPIDCommand extends Command {
  
  private PIDController DeflectorPID = new PIDController(
    Constants.DeflectorConstants.kP, 
    Constants.DeflectorConstants.kI, 
    Constants.DeflectorConstants.kD);

  ArmSubsystem armsubsystem;
  /** Creates a new ArmPIDCommand. */
  public DeflectorPIDCommand(Subsystem d_DeflectorSubsystem, double setpoint) {
    DeflectorPID.setSetpoint(setpoint);
    DeflectorPID.setTolerance(Constants.DeflectorConstants.DeflectorPIDTolerance);
    addRequirements(d_DeflectorSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double value = DeflectorPID.calculate(armsubsystem.getArmAngle());
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
    return DeflectorPID.atSetpoint();
  }
}
