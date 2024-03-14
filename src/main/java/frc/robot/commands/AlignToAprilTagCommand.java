package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LimelightBackSubsystem;
import frc.robot.subsystems.Swerve;

public class AlignToAprilTagCommand extends Command {
    private final LimelightBackSubsystem limelightSubsystem;
    private final Swerve driveSubsystem;
    private final PIDController pid = new PIDController(0.3, 0.2, 0.05);

    private final int pipelineBeforeRunning;

    public AlignToAprilTagCommand(LimelightBackSubsystem limelightSubsystem, Swerve driveSubsystem) {
        this.pipelineBeforeRunning = limelightSubsystem.getPipeline();
        limelightSubsystem.setPipeline(2);

        this.limelightSubsystem = limelightSubsystem;
        this.driveSubsystem = driveSubsystem;
        this.pid.setTolerance(.1);
        this.pid.setSetpoint(0);


        this.addRequirements(limelightSubsystem, driveSubsystem);
    }

    @Override
    public void execute() {

        double toRotate = this.pid.calculate(this.limelightSubsystem.getTargetX());

        this.driveSubsystem.drive(new Translation2d(0, new Rotation2d()), toRotate, false, true);

        // SmartDashboard.putNumber("Visible AprilTag", this.limelightSubsystem.getTID());
    }

    @Override
    public boolean isFinished() {
        return pid.atSetpoint();
    }

    @Override
    public void end(boolean interrupted) {
        this.limelightSubsystem.setPipeline(this.pipelineBeforeRunning);
    }
}
