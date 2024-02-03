package frc.robot;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.lib.util.COTSTalonFXSwerveConstants;
import frc.lib.util.COTSTalonFXSwerveConstants.SDS.MK4i.driveRatios;
import frc.lib.util.SwerveModuleConstants;
import frc.robot.commands.DeflectorPIDCommand;
import frc.robot.commands.ShootCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DeflectorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

import static frc.lib.util.COTSTalonFXSwerveConstants.SDS.MK4i.*;

public final class Constants {
     private final static ShooterSubsystem s_ShooterSubsystem = new ShooterSubsystem();
    private final static IntakeSubsystem i_IntakeSubsystem = new IntakeSubsystem();
    private final static DeflectorSubsystem d_DeflectorSubsystem = new DeflectorSubsystem();
    private final ArmSubsystem a_ArmSubsystem = new ArmSubsystem();
    private final LimelightSubsystem l_LimelightSubsystem = new LimelightSubsystem();
    public static final double CONTROLLER_DEADBAND = 0.1;

    public static final class Swerve {
        /**
         * Whether gyroscope values should be inverted.
         */
        public static final boolean INVERT_GYRO = true;

        /**
         * Constants for the motor setup that we're using.
         */
        public static final COTSTalonFXSwerveConstants FALCON_500_CONSTANTS = Falcon500(driveRatios.L2);

        /**
         * Units: Meters
         */
        public static final double TRACK_WIDTH = Units.inchesToMeters(28.5);

        /**
         * Units: Meters
         */
        public static final double BASE_WIDTH = Units.inchesToMeters(28.5);

        /**
         * Units: Meters
         */
        public static final double DRIVEBASE_DIAMETER = Math.sqrt(TRACK_WIDTH * TRACK_WIDTH + BASE_WIDTH * BASE_WIDTH);

        /**
         * Units: Meters
         */
        public static final double DRIVEBASE_RADIUS = DRIVEBASE_DIAMETER / 2f;

        public static final double WHEEL_CIRCUMFERENCE = FALCON_500_CONSTANTS.wheelCircumference;

        public static final SwerveDriveKinematics KINEMATICS = new SwerveDriveKinematics(
                new Translation2d(BASE_WIDTH / 2.0, TRACK_WIDTH / 2.0),
                new Translation2d(BASE_WIDTH / 2.0, -TRACK_WIDTH / 2.0),
                new Translation2d(-BASE_WIDTH / 2.0, TRACK_WIDTH / 2.0),
                new Translation2d(-BASE_WIDTH / 2.0, -TRACK_WIDTH / 2.0));

        /* Module Gear Ratios */
        public static final double DRIVE_GEAR_RATIO = FALCON_500_CONSTANTS.driveGearRatio;
        public static final double ANGLE_GEAR_RATIO = FALCON_500_CONSTANTS.angleGearRatio;

        public static final InvertedValue ANGLE_MOTOR_INVERT = FALCON_500_CONSTANTS.angleMotorInvert;
        public static final InvertedValue DRIVE_MOTOR_INVERT = FALCON_500_CONSTANTS.driveMotorInvert;

        public static final SensorDirectionValue CANCODER_INVERT = FALCON_500_CONSTANTS.cancoderInvert;

        /**
         * Units: Volts
         */
        public static final int ANGLE_CURRENT_LIMIT = 25;

        public static final int ANGLE_CURRENT_THRESHOLD = 40;
        public static final double ANGLE_CURRENT_THRESHOLD_TIME = 0.1;
        public static final boolean ANGLE_ENABLE_CURRENT_LIMIT = true;

        public static final int DRIVE_CURRENT_LIMIT = 35;
        public static final int DRIVE_CURRENT_THRESHOLD = 60;
        public static final double DRIVE_CURRENT_THRESHOLD_TIME = 0.1;
        public static final boolean DRIVE_ENABLE_CURRENT_LIMIT = true;

        /*
         * These values are used by the drive falcon to ramp in open loop and closed
         * loop driving.
         * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc
         */
        public static final double OPEN_LOOP_RAMP = 0.25;
        public static final double CLOSED_LOOP_RAMP = 0.0;

        public static final PIDConstants ANGLE_PID = new PIDConstants(FALCON_500_CONSTANTS.angleKP,
                FALCON_500_CONSTANTS.angleKI, FALCON_500_CONSTANTS.angleKD);
        public static final PIDConstants DRIVE_PID = new PIDConstants(0.12, 0.0, 0.0);

        /* Drive Motor Characterization Values From SYSID */
        public static final double DRIVE_KS = 0.32;
        public static final double DRIVE_KV = 1.51;
        public static final double DRIVE_KA = 0.27;

        /** Units: m/s */
        public static final double MAX_SPEED = 4.5;
        /** Units: radians/s */
        public static final double MAX_ANGULAR_VELOCITY = 10.0;

        /* Neutral Modes */
        public static final NeutralModeValue ANGLE_NEUTRAL_MODE = NeutralModeValue.Coast;
        public static final NeutralModeValue DRIVE_NEUTRAL_MODE = NeutralModeValue.Brake;

        /* Module Specific Constants */
        /* Front Left Module - Module 0 */
        public static final class Mod0 {
            public static final int driveMotorID = 8;
            public static final int angleMotorID = 7;
            public static final int canCoderID = 12;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(201.621);
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
                    canCoderID, angleOffset);
        }

        /* Front Right Module - Module 1 */
        public static final class Mod1 {
            public static final int driveMotorID = 2;
            public static final int angleMotorID = 1;
            public static final int canCoderID = 9;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(22.932);
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
                    canCoderID, angleOffset);
        }

        /* Back Left Module - Module 2 */
        public static final class Mod2 {
            public static final int driveMotorID = 6;
            public static final int angleMotorID = 5;
            public static final int canCoderID = 11;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(50.186);
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
                    canCoderID, angleOffset);
        }

        /* Back Right Module - Module 3 */
        public static final class Mod3 {
            public static final int driveMotorID = 4;
            public static final int angleMotorID = 3;
            public static final int canCoderID = 10;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(5.098);
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
                    canCoderID, angleOffset);
        }

        public static final HolonomicPathFollowerConfig PATHPLANNER_FOLLOWER_CONFIG = new HolonomicPathFollowerConfig(
                new PIDConstants(0.3, .02, .05),
                new PIDConstants(.5, .025, 0.05),
                MAX_SPEED,
                DRIVEBASE_RADIUS,
                new ReplanningConfig());

    }

    public static final class ArmConstants {
        public static final int ArmMotorID = 13;

        public static final double kP = 0.0;
        public static final double kI = 0.0;
        public static final double kD = 0.0;

        public static final double ArmPIDTolerance = 0.0;
        public static final double ArmPosInValue = 0.0;
        public static final double ArmPosOutValue = 0.0;

    }

    public static final class IntakeConstants {
        public static final int IntakeMotorID = 14;
    }

    public static final class DeflectorConstants {
        public static final int DeflectorMotorID = 15;

        public static final double kP = 0.0;
        public static final double kI = 0.0;
        public static final double kD = 0.0;

        public static final double DeflectorPIDTolerance = 0.0;
        public static final double DeflectorPosInValue = 0.0;
        public static final double DeflectorPosOutValue = 0.0;

    }

    public static final class AutoAimConstants {
        public static final double kP = 0.007237;
        public static final double kI = 0.000666;
        public static final double kD = 0.0000333;

        public static final double AutoAimPIDTolerance = 1.0;
        // public static final double DeflectorPosInValue = 0.0;
        // public static final double DeflectorPosOutValue = 0.0;

    }

    public static final class AutoFollowConstants {
        public static final double kP = 0.01571;
        public static final double kI = 0.000666;
        public static final double kD = 0.0000333;

        public static final double AutoFollowPIDTolerance = 1.0;
        // public static final double DeflectorPosInValue = 0.0;
        // public static final double DeflectorPosOutValue = 0.0;

    }

    public static final class AutoConstants { // TODO: The below constants are used in the example auto, and must be
                                              // tuned to specific robot
        public static final double kMaxSpeedMetersPerSecond = 3;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

        public static final double kPXController = 1;
        public static final double kPYController = 1;
        public static final double kPThetaController = 1;

        public static final SequentialCommandGroup ShootS = new SequentialCommandGroup(
                new ParallelCommandGroup(
                        new DeflectorPIDCommand(d_DeflectorSubsystem,
                                Constants.DeflectorConstants.DeflectorPosOutValue)), new ShootCommand(i_IntakeSubsystem, s_ShooterSubsystem),
                new DeflectorPIDCommand(d_DeflectorSubsystem, Constants.DeflectorConstants.DeflectorPosInValue));

        /**
         * Config for PathPlanner to follow auto paths
         */

        /* Constraint for the motion profilied robot angle controller */
        // public static final TrapezoidProfile.Constraints kThetaControllerConstraints
        // =
        // new TrapezoidProfile.Constraints(
        // kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
    }

}
