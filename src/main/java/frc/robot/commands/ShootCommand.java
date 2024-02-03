// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootCommand extends Command {
  IntakeSubsystem i_IntakeSubsystem;
  ShooterSubsystem s_ShooterSubsystem;
  double speed;
  double StartTime;
  double Time;

  /** Creates a new ShootCommand. */
  public ShootCommand(Subsystem i_IntakeSubsystem2, Subsystem s_ShooterSubsystem2) {
    // Use addRequirements() here to declare subsystem dependencies.

    addRequirements(i_IntakeSubsystem2);
    addRequirements(s_ShooterSubsystem2);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    new SequentialCommandGroup(
        s_ShooterSubsystem.runCmd(1),
        new WaitCommand(1),
        i_IntakeSubsystem.runCmd(1),
        new WaitCommand(1),
        new ParallelCommandGroup(s_ShooterSubsystem.runCmd(0), i_IntakeSubsystem.runCmd(0)));

    i_IntakeSubsystem.set(1.0);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
