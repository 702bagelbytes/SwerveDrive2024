package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import frc.robot.autos.*;
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

    /* Subsystems */
    private final Swerve s_Swerve = new Swerve();
    private final ArmSubsystem a_ArmSubsystem = new ArmSubsystem();
    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        s_Swerve.setDefaultCommand(
                new TeleopSwerve(
                        s_Swerve,
                        () -> Math.pow(-driver.getRawAxis(translationAxis) * power, 3),
                        () -> Math.pow(-driver.getRawAxis(strafeAxis) * power, 3),
                        () -> Math.pow(-driver.getRawAxis(rotationAxis) * power, 3),
                        () -> robotCentric.getAsBoolean()));

        a_ArmSubsystem.setDefaultCommand(a_ArmSubsystem.moveCmd(()-> codriver.getRawAxis(translationAxis)));
        // Configure the button bindings
        configureButtonBindings();
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
        zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroHeading()));
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
        // An ExampleCommand will run in autonomous
        return new exampleAuto(s_Swerve);
    }
}
