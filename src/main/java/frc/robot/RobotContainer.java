package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import frc.robot.commands.*;
import frc.robot.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    private final Subsystem s_ShooterSubsystem = new ShooterSubsystem();
    private final Subsystem i_IntakeSubsystem = new IntakeSubsystem();
    private final Subsystem d_DeflectorSubsystem = new DeflectorSubsystem();

    /* Controllers */
    private final Joystick driver = new Joystick(0);
    private final Joystick codriver = new Joystick(1);


    /* Drive Controls */
    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;

    /* Driver Buttons */
    private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kY.value);
    private final JoystickButton robotCentric = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);
    private final JoystickButton slowMode = new JoystickButton(driver, XboxController.Button.kA.value);
    private final JoystickButton fastMode = new JoystickButton(driver, XboxController.Button.kB.value);
    private final JoystickButton ArmPosIn = new JoystickButton(codriver, XboxController.Button.kA.value);
    private final JoystickButton ArmPosOut = new JoystickButton(codriver, XboxController.Button.kY.value);
    private final JoystickButton ShootS = new JoystickButton(codriver, XboxController.Button.kLeftBumper.value);
    private final JoystickButton ShootA = new JoystickButton(codriver, XboxController.Button.kLeftBumper.value);


    private double power = 1;

    private final SendableChooser<Command> autoChooser;

    /* Subsystems */
    private final Swerve s_Swerve = new Swerve();

    /**
     * The container for the robot. Contains subsystems, IO devices, and commands.
     */
    public RobotContainer() {
        field = new Field2d();
        SmartDashboard.putData("Field", field);

        // Logging callback for current robot pose
        PathPlannerLogging.setLogCurrentPoseCallback((pose) -> {
            field.setRobotPose(pose);
        });

        // Logging callback for target robot pose
        PathPlannerLogging.setLogTargetPoseCallback((pose) -> {
            field.getObject("target pose").setPose(pose);
        });

        // Logging callback for the active path, this is sent as a list of poses
        PathPlannerLogging.setLogActivePathCallback((poses) -> {
            field.getObject("path").setPoses(poses);
        });

        swerveDrive.setDefaultCommand(
                new TeleopSwerve(
                        swerveDrive,
                        () -> Math.pow(-driver.getRawAxis(translationAxis) * power, 3),
                        () -> Math.pow(-driver.getRawAxis(strafeAxis) * power, 3),
                        () -> Math.pow(-driver.getRawAxis(rotationAxis) * power, 3),
                        () -> robotCentric.getAsBoolean()));

        a_ArmSubsystem.setDefaultCommand(a_ArmSubsystem.moveCmd(()-> codriver.getRawAxis(translationAxis)));
        // Configure the button bindings
        configureButtonBindings();

        // Build an auto chooser. This will use Commands.none() as the default option.
        autoChooser = AutoBuilder.buildAutoChooser();

        SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
     * it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        /* Driver Buttons */
        zeroGyro.onTrue(new InstantCommand(() -> swerveDrive.zeroHeading()));
        slowMode.onTrue(new InstantCommand(() -> RobotContainer.this.power = .77));
        fastMode.onTrue(new InstantCommand(() -> RobotContainer.this.power = 1));
        ArmPosIn.onTrue(new ArmPIDCommand(a_ArmSubsystem, Constants.ArmConstants.ArmPosInValue));
        ArmPosOut.onTrue(new ArmPIDCommand(a_ArmSubsystem, Constants.ArmConstants.ArmPosOutValue));
        ShootS.onTrue(new ShootCommand(i_IntakeSubsystem, s_ShooterSubsystem));
        ShootA.onTrue(new SequentialCommandGroup(
            new ParallelCommandGroup(new ShootCommand(i_IntakeSubsystem, s_ShooterSubsystem), 
        new DeflectorPIDCommand(d_DeflectorSubsystem, Constants.DeflectorConstants.DeflectorPosOutValue)), 
        new DeflectorPIDCommand(d_DeflectorSubsystem, Constants.DeflectorConstants.DeflectorPosInValue))
        );

    }
    public void ResetArmPosition(){
        a_ArmSubsystem.ResetArmPos();
    }
    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return new SequentialCommandGroup((new InstantCommand(() -> {
            swerveDrive.zeroHeading();
            swerveDrive.gyro.reset();
        })), autoChooser.getSelected());
        // An ExampleCommand will run in autonomous
        // return new exampleAuto(s_Swerve);

    }
}
