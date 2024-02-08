package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;

public class TeleopSwerve extends Command {
    private Swerve swerveSubsystem;
    private DoubleSupplier translationSupplier;
    private DoubleSupplier strafeSupplier;
    private DoubleSupplier rotationSupplier;
    private BooleanSupplier robotCentricSupplier;

    public TeleopSwerve(Swerve s_Swerve, DoubleSupplier translationSup, DoubleSupplier strafeSup,
            DoubleSupplier rotationSup, BooleanSupplier robotCentricSup) {
        this.swerveSubsystem = s_Swerve;
        addRequirements(s_Swerve);

        this.translationSupplier = translationSup;
        this.strafeSupplier = strafeSup;
        this.rotationSupplier = rotationSup;
        this.robotCentricSupplier = robotCentricSup;
    }

    @Override
    public void execute() {
        /* Get Values, Deadband */
        double translationVal = MathUtil.applyDeadband(translationSupplier.getAsDouble(),
                Constants.CONTROLLER_DEADBAND);
        double strafeVal = MathUtil.applyDeadband(strafeSupplier.getAsDouble(), Constants.CONTROLLER_DEADBAND);
        double rotationVal = MathUtil.applyDeadband(rotationSupplier.getAsDouble(), Constants.CONTROLLER_DEADBAND);

        /* Drive */
        swerveSubsystem.drive(
                new Translation2d(translationVal, strafeVal).times(Constants.Swerve.MAX_SPEED),
                rotationVal * Constants.Swerve.MAX_ANGULAR_VELOCITY,
                !robotCentricSupplier.getAsBoolean(),
                true);
    }
}