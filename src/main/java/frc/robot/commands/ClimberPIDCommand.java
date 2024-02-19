// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.Constants;
import frc.robot.subsystems.ClimberSubsystem;


public class ClimberPIDCommand extends Command {

  private PIDController LeftLiftPID = new PIDController(
      Constants.ClimberConstants.LP,
      Constants.ClimberConstants.LI,
      Constants.ClimberConstants.LD);

  private PIDController RightLiftPID = new PIDController(
      Constants.ClimberConstants.RP,
      Constants.ClimberConstants.RI,
      Constants.ClimberConstants.RD);

  ClimberSubsystem c_ClimberSubsystem;

  /** Creates a new ArmPIDCommand. */
  public ClimberPIDCommand(ClimberSubsystem c_ClimberSubsystem, double Leftsetpoint, double Rightsetpoint) {
    this.c_ClimberSubsystem = c_ClimberSubsystem;
    LeftLiftPID.setSetpoint(Leftsetpoint);
    LeftLiftPID.setTolerance(Constants.ClimberConstants.LiftPIDTolerance);
    RightLiftPID.setSetpoint(Rightsetpoint);
    RightLiftPID.setTolerance(Constants.ClimberConstants.LiftPIDTolerance);
    addRequirements(c_ClimberSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double Lvalue = LeftLiftPID.calculate(c_ClimberSubsystem.getLmotorPos());
    double Rvalue = RightLiftPID.calculate(c_ClimberSubsystem.getRmotorPos());
    c_ClimberSubsystem.setLmotor(Lvalue);
    c_ClimberSubsystem.setRmotor(Rvalue);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    c_ClimberSubsystem.setLmotor(0);
    c_ClimberSubsystem.setRmotor(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return LeftLiftPID.atSetpoint() && RightLiftPID.atSetpoint();
  }
}
