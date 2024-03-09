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
import frc.lib.util.COTSTalonFXSwerveConstants;
import frc.lib.util.COTSTalonFXSwerveConstants.SDS.MK4i.driveRatios;
import frc.lib.util.SwerveModuleConstants;

import static frc.lib.util.COTSTalonFXSwerveConstants.SDS.MK4i.*;

public final class Constants {
    public static final double CONTROLLER_DEADBAND = 0.1;

    public enum Direction {
        UP(0), RIGHT(90), DOWN(180), LEFT(270);

        int direction;

        private Direction(int direction) {
            this.direction = direction;
        }

        @Override
        public String toString() {
            switch (this) {
                case UP:
                    return "UP";
                case DOWN:
                    return "DOWN";
                case LEFT:
                    return "LEFT";
                case RIGHT:
                    return "RIGHT";
            }

            return null;
        }
    }

    public enum ShooterSpeeds {
        STOPPED(0,   "Stopped         "),
        SLOW(.30,     "Slow               "),
        MEDIUM(.45,  "Medium         "),
        DEFAULT(.52, "Default           "),
        VERY_FAST(.62, "Very Fast       "),
        INSANELY_FAST(.75, "Insanely Fast");

        public final double speed;
        public final String label;
        private ShooterSpeeds(double speed, String label) {
            this.speed = speed;
            this.label = label;
        }

        public ShooterSpeeds next() {
            switch (this) {
                case STOPPED:
                    return SLOW;
                case SLOW:
                    return MEDIUM;
                case MEDIUM:
                    return DEFAULT;
                case DEFAULT:
                    return VERY_FAST;
                case VERY_FAST:
                    return INSANELY_FAST;
                case INSANELY_FAST:
                    return INSANELY_FAST;
            }

            throw new IllegalStateException("never reached");
        }

        public ShooterSpeeds prev() {
            switch (this) {
                case STOPPED:
                    return STOPPED;
                case SLOW:
                    return STOPPED;
                case MEDIUM:
                    return SLOW;
                case DEFAULT:
                    return MEDIUM;
                case VERY_FAST:
                    return DEFAULT;
                case INSANELY_FAST:
                    return VERY_FAST;
            }

            // should never be reached
            throw new IllegalStateException("never reached");
        }
    }

    /**
     * Corresponds to port zero on the Roborio DIO. 
     */
    public static final int LIMIT_SWITCH_INTAKE = 0;

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
        public static final double TRACK_WIDTH = Units.inchesToMeters(23.5);

        /**
         * Units: Meters
         */
        public static final double BASE_WIDTH = Units.inchesToMeters(23.5);

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
        public static final int ANGLE_STATOR_CURRENT_LIMIT = 40;
        public static final int ANGLE_CURRENT_LIMIT = 25;
        public static final int ANGLE_CURRENT_THRESHOLD = 40;
        public static final double ANGLE_CURRENT_THRESHOLD_TIME = 0.1;
        public static final boolean ANGLE_ENABLE_CURRENT_LIMIT = true;
        public static final boolean ANGLE_ENABLE_STATOR_CURRENT_LIMIT = false;

        public static final int DRIVE_STATOR_CURRENT_LIMIT = 50;
        public static final int DRIVE_CURRENT_LIMIT = 35;//35
        public static final int DRIVE_CURRENT_THRESHOLD = 50;//60
        public static final double DRIVE_CURRENT_THRESHOLD_TIME = 0.1;
        public static final boolean DRIVE_ENABLE_CURRENT_LIMIT = true;
        public static final boolean DRIVE_ENABLE_STATOR_CURRENT_LIMIT = false;

        /*
         * These values are used by the drive falcon to ramp in open loop and closed
         * loop driving.
         * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc
         */
        public static final double OPEN_LOOP_RAMP = 0.45;
        public static final double CLOSED_LOOP_RAMP = 0;

        public static final PIDConstants ANGLE_PID = new PIDConstants(FALCON_500_CONSTANTS.angleKP,
                FALCON_500_CONSTANTS.angleKI, FALCON_500_CONSTANTS.angleKD);
        public static final PIDConstants DRIVE_PID = new PIDConstants(0.12, 0.0, 0.0);

        /* Drive Motor Characterization Values From SYSID */
        public static final double DRIVE_KS = 0.32;
        public static final double DRIVE_KV = 1.51;
        public static final double DRIVE_KA = 0.27;

        /** Units: m/s */
        public static final double MAX_SPEED = 10;
        /** Units: radians/s */
        public static final double MAX_ANGULAR_VELOCITY = 10.0;

        /* Neutral Modes */
        public static final NeutralModeValue ANGLE_NEUTRAL_MODE = NeutralModeValue.Coast;
        public static final NeutralModeValue DRIVE_NEUTRAL_MODE = NeutralModeValue.Brake;

        /* Module Specific Constants */
        /* Front Left Module - Module 0 */
        public static final class Mod0 {
            public static final int driveMotorID = 10;
            public static final int angleMotorID = 11;
            public static final int canCoderID = 12;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(335.478);
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
                    canCoderID, angleOffset);
        }

        /* Front Right Module - Module 1 */
        public static final class Mod1 {
            public static final int driveMotorID = 7;
            public static final int angleMotorID = 8;
            public static final int canCoderID = 9;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(-191.25);
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
                    canCoderID, angleOffset);
        }

        /* Back Left Module - Module 2 */
        public static final class Mod2 {
            public static final int driveMotorID = 1;
            public static final int angleMotorID = 2;
            public static final int canCoderID = 3;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(50.8007);
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
                    canCoderID, angleOffset);
        }

        /* Back Right Module - Module 3 */
        public static final class Mod3 {
            public static final int driveMotorID = 4;
            public static final int angleMotorID = 5;
            public static final int canCoderID = 6;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(-166.648);
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
                    canCoderID, angleOffset);
        }

        public static final HolonomicPathFollowerConfig PATHPLANNER_FOLLOWER_CONFIG = new HolonomicPathFollowerConfig(
                new PIDConstants(1.14, .03, .15),
                new PIDConstants(.97, .035, 0.15),
                MAX_SPEED,
                DRIVEBASE_RADIUS,
                new ReplanningConfig());

    }

    public static final class ArmConstants {
        public static final int ArmMotorID = 13;

        public static final double kP = 0.042;
        public static final double kI = 0.0014;
        public static final double kD = 0.0042;

        public static final double ArmPIDTolerance = 1.0;
        public static final double ArmPosInValue = 0.0;
        public static final double ArmPosOutValue = -58;
        public static final boolean ArmLimitEnable = true;

        public static final int STATOR_CURRENT_LIMIT = 20;
        public static final int CURRENT_LIMIT = 15;
        public static final int CURRENT_THRESHOLD = 20;
        public static final double CURRENT_THRESHOLD_TIME = 0.1;
        public static final boolean ENABLE_CURRENT_LIMIT = true;
        public static final boolean ENABLE_STATOR_CURRENT_LIMIT = true;

    }

    public static final class IntakeConstants {
        public static final int IntakeMotorID = 14;
        
        public static final double MaxIntakeSpeed = 1;
    }

    public static final class DeflectorConstants {
        public static final int DeflectorMotorID = 15;
        public static final boolean DeflectorMotorInverted = false;


        public static final double kP = 0.028;
        public static final double kI = 0.00;
        public static final double kD = 0.0012;

        public static final double DeflectorPIDTolerance = 0.5;
        public static final double DeflectorPosInValue = 0.0;
        public static final double DeflectorPosOutValue = 105.0;  
        public static final double DeflectorPosStowValue = 120.0;  

        public static final boolean DeflectorLimitEnable = true;


    }

    public static final class ShootConstants {
        public static final int BottomShootMotorID = 17;
        public static final int TopShootMotorID = 16;

        public static final boolean TopShootMotorInverted = true;
        public static final boolean BottomShootMotorInverted = false;

        public static final NeutralModeValue TopShootMotorMode = NeutralModeValue.Coast;
        public static final NeutralModeValue BottomShootMotorMode = NeutralModeValue.Coast;
        
        public static final double MaxShootSpeed = 1;
    }

    public static final class ClimberConstants {
        public static final int LeftLiftMotorID = 18;
        public static final int RightLiftMotorID = 19;

         public static final boolean LeftLiftMotorInverted = true;
        public static final boolean RightLiftMotorInverted = true;

        public static final NeutralModeValue LeftLiftMotorMode = NeutralModeValue.Brake;
        public static final NeutralModeValue RightLiftMotorMode = NeutralModeValue.Brake;
        
        public static final double MaxLiftSpeed = 1.0;

        public static final double LP = 0.05;
        public static final double LI = 0.0023;
        public static final double LD = 0.00147;
        public static final double RP = 0.1;
        public static final double RI = 0.0023;
        public static final double RD = 0.00147;

        public static final boolean LiftLimitEnable = true;
        public static final double LiftPIDTolerance = .5;
        public static final double LeftLiftPosInValue = -41.7;
        public static final double LeftLiftPosOutValue = 0;   
        public static final double RightLiftPosInValue = -13;
        public static final double RightLiftPosOutValue = 0;     

    }

    public static final class ShooterConstants{
        public static final int TMotorID = 17;
        public static final int BMotorID = 16;

        public static final boolean TMotorInvert = true;
        public static final boolean BMotorInvert = false;
    }

    public static final class AutoAimConstants {
        public static final double kP = 0.005837;
        public static final double kI = 0.0000665;
        public static final double kD = 0.0003333;

        public static final double AutoAimPIDTolerance = 1.0;
        // public static final double DeflectorPosInValue = 0.0;
        // public static final double DeflectorPosOutValue = 0.0;

    }

    public static final class AutoFollowConstants {
        public static final double kP = 0.04571;
        public static final double kI = 0.000665;
        public static final double kD = 0.001333;

        public static final double AutoFollowPIDTolerance = 1.0;
        // public static final double DeflectorPosInValue = 0.0;
        // public static final double DeflectorPosOutValue = 0.0;

    }

    public static final class AutoConstants { // TODO: The below constants are used in the example auto, and must be
                                              // tuned to specific robot
        public static final double kMaxSpeedMetersPerSecond = 7.5;
        public static final double kMaxAccelerationMetersPerSecondSquared = 5;
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

        public static final double kPXController = 1;
        public static final double kPYController = 1;
        public static final double kPThetaController = 1;

        
        /**
         * Config for PathPlanner to follow auto paths
         */

        /* Constraint for the motion profilied robot angle controller */
        // public static final TrapezoidProfile.Constraints kThetaControllerConstraints
        // =
        // new TrapezoidProfile.Constraints(
        // kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
    }

    public static final class LEDConstants {
        public static final int LED_1_PwmID = 9;

        public static final int LED_1_Length = 28;
    }
}
