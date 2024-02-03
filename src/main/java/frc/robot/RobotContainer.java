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
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import frc.robot.commands.*;
import frc.robot.subsystems.*;
//import frc.robot.commands.AutoAimCommand;
//import frc.robot.commands.AutoFollowCommand;

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
    private final static ShooterSubsystem s_ShooterSubsystem = new ShooterSubsystem();
    private final static IntakeSubsystem i_IntakeSubsystem = new IntakeSubsystem();
    private final static DeflectorSubsystem d_DeflectorSubsystem = new DeflectorSubsystem();
    private final static ArmSubsystem a_ArmSubsystem = new ArmSubsystem();
    private final static LimelightSubsystem l_LimelightSubsystem = new LimelightSubsystem();

    public SequentialCommandGroup ShootACommand = new SequentialCommandGroup(
                new ParallelCommandGroup(
                        new DeflectorPIDCommand(d_DeflectorSubsystem,
                                Constants.DeflectorConstants.DeflectorPosOutValue)), new ShootCommand(i_IntakeSubsystem, s_ShooterSubsystem),
                new DeflectorPIDCommand(d_DeflectorSubsystem, Constants.DeflectorConstants.DeflectorPosInValue));

    public Command ShootSCommand = new ShootCommand(i_IntakeSubsystem, s_ShooterSubsystem);

    public Command IntakeIn = new ArmPIDCommand(a_ArmSubsystem, Constants.ArmConstants.ArmPosInValue);
    public Command IntakeOut = new ArmPIDCommand(a_ArmSubsystem, Constants.ArmConstants.ArmPosOutValue);

    public Command IntakeOn = new InstantCommand(()->i_IntakeSubsystem.runCmd(1));
    public Command IntakeOff = new InstantCommand(()->i_IntakeSubsystem.runCmd(0));

    /* Controllers */
    private final Joystick driver = new Joystick(0);
    private final Joystick codriver = new Joystick(1);

    /* Drive Controls */
    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;
    private final int LeftTrigger = XboxController.Axis.kLeftTrigger.value;
    private final int RightTrigger = XboxController.Axis.kRightTrigger.value;

    /* Driver Buttons */
    private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kY.value);
    private final JoystickButton robotCentric = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);
    private final JoystickButton slowMode = new JoystickButton(driver, XboxController.Button.kA.value);
    private final JoystickButton fastMode = new JoystickButton(driver, XboxController.Button.kB.value);
    private final JoystickButton ArmPosIn = new JoystickButton(codriver, XboxController.Button.kA.value);
    private final JoystickButton ArmPosOut = new JoystickButton(codriver, XboxController.Button.kY.value);
    private final JoystickButton ShootS = new JoystickButton(codriver, XboxController.Button.kLeftBumper.value);
    private final JoystickButton ShootA = new JoystickButton(codriver, XboxController.Button.kLeftBumper.value);
    public final JoystickButton AutoAim = new JoystickButton(driver, XboxController.Button.kStart.value);
    public final JoystickButton AutoTurn = new JoystickButton(driver, XboxController.Button.kX.value);
    public final JoystickButton Intake = new JoystickButton(codriver, XboxController.Axis.kLeftTrigger.value);
    public final JoystickButton Outtake = new JoystickButton(codriver, XboxController.Axis.kRightTrigger.value);

    private double power = 1;

    public static double AimPID = 0;
    public static double FollowPID = 0;
    public static String Color = "blue";

    private final SendableChooser<Command> autoChooser;
    private final SendableChooser<Command> teamChooser;
    
    /* Subsystems */
    private final Swerve s_Swerve = new Swerve();

    /**
     * The container for the robot. Contains subsystems, IO devices, and commands.
     */
    public RobotContainer() {
        Field2d field = new Field2d();
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

        s_Swerve.setDefaultCommand(
                new TeleopSwerve(
                        s_Swerve,
                        () -> Math.pow(-driver.getRawAxis(translationAxis) * power, 3) + FollowPID,
                        () -> Math.pow(-driver.getRawAxis(strafeAxis) * power, 3),
                        () -> Math.pow(-driver.getRawAxis(rotationAxis) * power, 3) + AimPID,
                        () -> robotCentric.getAsBoolean()));

        a_ArmSubsystem.setDefaultCommand(a_ArmSubsystem.moveCmd(() -> codriver.getRawAxis(translationAxis)));

        i_IntakeSubsystem.setDefaultCommand(i_IntakeSubsystem.moveCmd(()-> codriver.getRawAxis(LeftTrigger) - codriver.getRawAxis(RightTrigger)));
        // Configure the button bindings
        configureButtonBindings();

        // Build an auto chooser. This will use Commands.none() as the default option.
        autoChooser = AutoBuilder.buildAutoChooser();
        teamChooser = new SendableChooser<>();
        teamChooser.addOption("Red", new InstantCommand(()-> Color = "red"));
        teamChooser.addOption("Blue", new InstantCommand(()-> Color = "blue"));

        SmartDashboard.putData("Auto Chooser", autoChooser);
        SmartDashboard.putData("Team Chooser", teamChooser);
        

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
        ArmPosIn.onTrue(IntakeIn);
        ArmPosOut.onTrue(IntakeOut);
        ShootS.onTrue(ShootSCommand);
        ShootA.onTrue(ShootACommand);
        
        
        
        AutoAim.whileTrue(new ParallelCommandGroup(new AutoFollowCommand(()->l_LimelightSubsystem.getTargetA(), ()->l_LimelightSubsystem.IsTargetAvailable()), new AutoAimCommand(()->l_LimelightSubsystem.getTargetX(), ()->l_LimelightSubsystem.IsTargetAvailable())));
        
        
        AutoAim.onFalse(new ParallelCommandGroup(new InstantCommand(() -> FollowPID = 0), new InstantCommand(() -> AimPID = 0)));
      
       

    }

    public void ResetArmPosition() {
        a_ArmSubsystem.ResetArmPos();
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return new SequentialCommandGroup((new InstantCommand(() -> {
            s_Swerve.zeroHeading();
            s_Swerve.gyro.reset();
        })), autoChooser.getSelected());
        // An ExampleCommand will run in autonomous
        // return new exampleAuto(s_Swerve);

    }
}
