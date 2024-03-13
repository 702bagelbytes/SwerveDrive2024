package frc.robot;



import java.util.function.BooleanSupplier;


import com.ctre.phoenix6.signals.NeutralModeValue;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;

import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.Constants.Direction;
import frc.robot.Constants.ShooterSpeeds;
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
    private final static ShooterSubsystem s_ShooterSubsystem = new ShooterSubsystem();
    private final static IntakeSubsystem i_IntakeSubsystem = new IntakeSubsystem();
    private final static DeflectorSubsystem d_DeflectorSubsystem = new DeflectorSubsystem();
    private final static ArmSubsystem a_ArmSubsystem = new ArmSubsystem();
    private final static LimelightSubsystem l_LimelightSubsystem = new LimelightSubsystem();
    private final static LimelightBackSubsystem l_LimelightBackSubsystem = new LimelightBackSubsystem();
    private final static LimitSwitch l_LimitSwitch = new LimitSwitch();
    private final static ClimberSubsystem c_ClimberSubsystem = new ClimberSubsystem();
    private final static LEDSubsystem l_LEDSubsystem = new LEDSubsystem();
    public static boolean isRingIn;

    /**
     * Stows the arm mechanism
     */

    public Command IntakeIn() {
        return new ArmPIDCommand(a_ArmSubsystem, Constants.ArmConstants.ArmPosInValue);
    }

    /**
     * Sticks out the arm mechanism
     */
    public Command IntakeOut() {
        return new ArmPIDCommand(a_ArmSubsystem, Constants.ArmConstants.ArmPosOutValue);
    }

    public Command DeflectorIn() {
        return new DeflectorPIDCommand(d_DeflectorSubsystem, Constants.DeflectorConstants.DeflectorPosInValue);
    }

    public Command DeflectorOut() {
        return new DeflectorPIDCommand(d_DeflectorSubsystem, Constants.DeflectorConstants.DeflectorPosOutValue);
    }

    public Command ArmMove(double value) {
        return new InstantCommand(() -> a_ArmSubsystem.set(value));
    }

    public Command DeflectorMove(double value) {
        return new InstantCommand(() -> d_DeflectorSubsystem.set(value));
    }

    /**
     * Turns on the intake motor
     */

    public Command IntakeOn(double val) {
        return Commands.runEnd(() -> i_IntakeSubsystem.set(val), () -> i_IntakeSubsystem.set(0), i_IntakeSubsystem)
                .until(l_LimitSwitch::isRingIn);
    }

    public Command AutoPickUp(double turn) {
        return new SequentialCommandGroup(IntakeOut(),
                (IntakeOn(0.40)).raceWith(
                new AutoFollowCommand(() -> l_LimelightSubsystem.getTargetX(),
                        () -> l_LimelightSubsystem.getTargetA(),
                        () -> l_LimelightSubsystem.IsTargetAvailable(), s_Swerve, turn)));
                
                
    }

    public Command AutoAmpScore(LimelightBackSubsystem l_LimelightBackSubsystem){
        Number Id[] = {1, 2};
        l_LimelightBackSubsystem.setId(Id);
        boolean x = (l_LimelightBackSubsystem.getTargetX() > -20 && l_LimelightBackSubsystem.IsTargetAvailable() == true);
        BooleanSupplier interrupt = ()-> x;
        
        return new SequentialCommandGroup(new ParallelCommandGroup(new AutoRotateCommand(-90, s_Swerve, -.4, 0).until(interrupt), Commands.runOnce(() -> s_ShooterSubsystem.set(0.42, 37), s_ShooterSubsystem)), Outtake(), Commands.runOnce(() -> s_ShooterSubsystem.set(0, 0)));
        
    }

    

    /**
     * Turns off the intake motor
     */
    public Command IntakeOff() {
        return new InstantCommand(() -> i_IntakeSubsystem.set(0));
    }

    public Command OnAndStow() {
        return Commands.either(IntakeIn(), i_IntakeSubsystem.runEndCmd(Constants.IntakeConstants.MaxIntakeSpeed),
                () -> l_LimitSwitch.isRingIn());
    }

    public Command Stow() {
        return Commands.either(new InstantCommand(), IntakeIn(),
                () -> l_LimitSwitch.isRingIn());
    }

    

    private double power = 1;

    private ShooterSpeeds topShooterSpeed = ShooterSpeeds.DEFAULT;
    private ShooterSpeeds bottomShooterSpeed = ShooterSpeeds.DEFAULT;

    public Command Outtake() {
        return new SequentialCommandGroup(
                Commands.runOnce(() -> i_IntakeSubsystem.setMode(NeutralModeValue.Coast), i_IntakeSubsystem),
                Commands.runOnce(() -> i_IntakeSubsystem.set(-0.55), i_IntakeSubsystem),
                new WaitCommand(0.2),
                Commands.runOnce(() -> i_IntakeSubsystem.set(0), i_IntakeSubsystem),
                Commands.runOnce(() -> i_IntakeSubsystem.setMode(NeutralModeValue.Brake), i_IntakeSubsystem));
    }
    public Command Shoot(double TopSpeed, double BottomSpeed) {
        return new SequentialCommandGroup(IntakeIn(),
                Commands.runOnce(() -> s_ShooterSubsystem.set(TopSpeed, BottomSpeed),
                        s_ShooterSubsystem),
                new WaitCommand(.52),
                Commands.runOnce(() -> i_IntakeSubsystem.set(-0.55), i_IntakeSubsystem),
                new WaitCommand(0.35),
                new ParallelCommandGroup(Commands.runOnce(() -> s_ShooterSubsystem.set(0), s_ShooterSubsystem),
                        Commands.runOnce(() -> i_IntakeSubsystem.set(0), i_IntakeSubsystem)));
    }

    public Command ShootA() {
        return new SequentialCommandGroup(IntakeIn(),
                Shoot(0.32, 0.39) //0.13 0.57   0.52 0    0.1542 0.42

        );
    }

    public Command ShootACommand() {
        return new SequentialCommandGroup(
                new ParallelCommandGroup(DeflectorOut(), ShootA()),new WaitCommand(0.4),
                DeflectorIn());
    }

    public Command ShootOn(double top, double Bottom){
        return Commands.run(() -> s_ShooterSubsystem.set(top, Bottom), s_ShooterSubsystem);
    }

    public Command ShootOff(){
        return new InstantCommand(() -> s_ShooterSubsystem.set(0, 0));
    }

    public Command QuickShoot(){
        return new SequentialCommandGroup(ShootOn(0.9, 0.9), new WaitCommand(0.2), Outtake(), ShootOff());
    }

    public Command DeflectorIn = new DeflectorPIDCommand(d_DeflectorSubsystem,
            Constants.DeflectorConstants.DeflectorPosInValue);
    public Command DeflectorOut = new DeflectorPIDCommand(d_DeflectorSubsystem,
            Constants.DeflectorConstants.DeflectorPosOutValue);

    /* Controllers */
    private final Joystick driver = new Joystick(0);
    private final Joystick codriver = new Joystick(1);
    //private final Joystick master = new Joystick(2);

    /* Drive Controls */
    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;
    private final int upAxis = XboxController.Axis.kRightY.value;
    private final int LeftTrigger = XboxController.Axis.kLeftTrigger.value;
    private final int RightTrigger = XboxController.Axis.kRightTrigger.value;

    /* Driver Buttons */
    private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kY.value);
    //private final JoystickButton robotCentric = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);
    private final JoystickButton slowMode = new JoystickButton(driver, XboxController.Button.kA.value);
    private final JoystickButton fastMode = new JoystickButton(driver, XboxController.Button.kB.value);
    private final JoystickButton ArmPosIn = new JoystickButton(codriver, XboxController.Button.kA.value);
    private final JoystickButton ArmPosOut = new JoystickButton(codriver, XboxController.Button.kY.value);
    private final JoystickButton ShootS = new JoystickButton(codriver, XboxController.Button.kLeftBumper.value);
    
    private final JoystickButton ShootA = new JoystickButton(codriver, XboxController.Button.kRightBumper.value);
    // public final JoystickButton AutoAim = new JoystickButton(driver,
    // XboxController.Button.kStart.value);
    public final JoystickButton AutoTurn = new JoystickButton(driver, XboxController.Button.kX.value);
    public final JoystickButton Intake = new JoystickButton(codriver, XboxController.Axis.kLeftTrigger.value);
    public final JoystickButton Outtake = new JoystickButton(codriver, XboxController.Axis.kRightTrigger.value);
    public final JoystickButton AutoShoot = new JoystickButton(codriver, XboxController.Button.kStart.value);
    public final JoystickButton AutoAim = new JoystickButton(driver, XboxController.Button.kStart.value);

    private final JoystickButton DeflectorPosIn = new JoystickButton(codriver, XboxController.Button.kB.value);
    private final JoystickButton DeflectorPosOut = new JoystickButton(codriver, XboxController.Button.kX.value);
    private final JoystickButton LiftPosOut = new JoystickButton(codriver, XboxController.Button.kLeftStick.value);
    private final JoystickButton LiftPosIn = new JoystickButton(codriver, XboxController.Button.kRightStick.value);

    private final JoystickButton AutoAmp = new JoystickButton(driver, XboxController.Button.kRightBumper.value);

    public final JoystickButton onandstow = new JoystickButton(driver, XboxController.Button.kX.value);

    private final POVButton Up = new POVButton(driver, Direction.UP.direction);
    // private final POVButton Up_Right = new POVButton(driver, Direction.UP_LEFT);
    // private final POVButton Up_Left = new POVButton(driver, Direction.LEFT.direction);

    private final POVButton Down = new POVButton(driver, Direction.DOWN.direction);
    // private final POVButton Down_Right = new POVButton(driver, Direction.RIGHT.direction);
    // private final POVButton Down_Left = new POVButton(driver, Direction.LEFT.direction);

    private final POVButton Right = new POVButton(driver, Direction.RIGHT.direction);
    private final POVButton Left = new POVButton(driver, Direction.LEFT.direction);

    private final POVButton OutIntake = new POVButton(codriver, Direction.UP.direction);
    private final POVButton InIntake = new POVButton(codriver, Direction.DOWN.direction);

    private final POVButton InDeflector = new POVButton(codriver, Direction.RIGHT.direction);
    private final POVButton OutDeflector = new POVButton(codriver, Direction.LEFT.direction);

    // private final POVButton increaseTopSpeed = new POVButton(master, Direction.UP.direction);
    // private final POVButton decreaseTopSpeed = new POVButton(master, Direction.DOWN.direction);

    // private final POVButton increaseBottomSpeed = new POVButton(master, Direction.RIGHT.direction);
    // private final POVButton decreaseBottomSpeed = new POVButton(master, Direction.LEFT.direction);

    public static double AimPID;
    public static double FollowPID;
    public static Color color = Color.kBlue;
    public static boolean robotCentric = false;

    private final SendableChooser<Command> autoChooser;
    private final SendableChooser<Command> teamChooser;

    // private static final Orchestra orchestra = new Orchestra("mario.chrp");

    /* Subsystems */
    private final Swerve s_Swerve = new Swerve();

    public void debugSpeeds() {
        SmartDashboard.putString("Top Shooter Speed",
                String.format("%s (%d%%)", topShooterSpeed.label, (int) (topShooterSpeed.speed * 100)));
        SmartDashboard.putString("Bottom Shooter Speed",
                String.format("%s (%d%%)", bottomShooterSpeed.label, (int) (bottomShooterSpeed.speed * 100)));
    }

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

        NamedCommands.registerCommand("ShootOn", Commands.runOnce(() -> s_ShooterSubsystem.set(0.52, 0.42)));
        NamedCommands.registerCommand("ShootOff", Commands.runOnce(() -> s_ShooterSubsystem.set(0, 0)));
        NamedCommands.registerCommand("Shoot", Shoot(0.58, 0.5));
        NamedCommands.registerCommand("Shoot2", Shoot(0.90, 0.21));
        NamedCommands.registerCommand("QuickShoot", QuickShoot());
        NamedCommands.registerCommand("IntakeOut", IntakeOut());
        NamedCommands.registerCommand("IntakeOff", IntakeOff());
        NamedCommands.registerCommand("IntakeOn", IntakeOn(0.6));
        NamedCommands.registerCommand("IntakeIn", IntakeIn());
         NamedCommands.registerCommand("Outtake", Outtake());
         NamedCommands.registerCommand("AutoPickUpCmdL", AutoPickUp(0.3));
        NamedCommands.registerCommand("AutoPickUpCmd", AutoPickUp(0));
        NamedCommands.registerCommand("AutoPickUpCmdR", AutoPickUp(-0.3));
        NamedCommands.registerCommand("AutoFollowCmd", new AutoFollowCommand(
                        () -> l_LimelightSubsystem.getTargetX(),
                        () -> l_LimelightSubsystem.getTargetA(),
                        () -> l_LimelightSubsystem.IsTargetAvailable(), s_Swerve, 0.3));

        s_Swerve.setDefaultCommand(
                new TeleopSwerve(
                        s_Swerve,
                        () -> -driver.getRawAxis(translationAxis) * power,
                        () -> -driver.getRawAxis(strafeAxis) * power,
                        () -> -driver.getRawAxis(rotationAxis) * power,
                        ()-> robotCentric));

        c_ClimberSubsystem.setDefaultCommand(c_ClimberSubsystem.moveCmd(() -> codriver.getRawAxis(translationAxis),
                () -> codriver.getRawAxis(upAxis)));
        i_IntakeSubsystem.setDefaultCommand(
                i_IntakeSubsystem
                        .moveCmd(() -> l_LimitSwitch.isRingIn() ? 0 - codriver.getRawAxis(RightTrigger) * 0.45
                                : codriver.getRawAxis(LeftTrigger) * 0.25 - codriver.getRawAxis(RightTrigger) * 0.45));

        
        l_LEDSubsystem.setDefaultCommand(new InstantCommand(() -> {
            if(l_LimitSwitch.isRingIn()) {
               l_LEDSubsystem.setColor(Color.kGreen);
               l_LEDSubsystem.setColor(Color.kWhite);
               new WaitCommand(0.1);
               l_LEDSubsystem.setColor(Color.kGreen);
               new WaitCommand(0.1);
               l_LEDSubsystem.setColor(Color.kWhite);
               new WaitCommand(0.1);
               l_LEDSubsystem.setColor(Color.kGreen);
            } else {
                l_LEDSubsystem.DoTheRainbow(true);;
            }
        }, l_LEDSubsystem
        ));

        //
        // l_LimitSwitch.setDefaultCommand(IsRingIn());
        // Configure the button bindings

        debugSpeeds();

        configureButtonBindings();

        // Build an auto chooser. This will use Commands.none() as the default option.
        autoChooser = AutoBuilder.buildAutoChooser();
        teamChooser = new SendableChooser<>();
        teamChooser.addOption("Red", new InstantCommand(() -> {
            color = Color.kRed;
            l_LEDSubsystem.setColor(color);
        }));
        teamChooser.addOption("Blue", new InstantCommand(() -> {
            color = Color.kBlue;
            l_LEDSubsystem.setColor(color);
        }));

        SmartDashboard.putData("Auto Chooser", autoChooser);
        SmartDashboard.putData("Team Chooser", teamChooser);
    }

    /**
     * If the limit switch is pressed, we can assume that the ring is inside!
     * 
     * @return the value of the limit switch.
     */

    public static void setAimPID(double AimPID) {
        RobotContainer.AimPID = AimPID;
    }

    public static void setFollowPID(double FollowPID) {
        RobotContainer.FollowPID = FollowPID;
    }

    private Command wrapSpeedChange(Runnable r) {
        return Commands.runOnce(() -> {
            r.run();
            RobotContainer.this.debugSpeeds();
        });
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
        slowMode.onTrue(new InstantCommand(() -> RobotContainer.this.power = .333));
        fastMode.onTrue(new InstantCommand(() -> RobotContainer.this.power = 1));
        ArmPosIn.onTrue(new ParallelCommandGroup(new ArmPIDCommand(a_ArmSubsystem, Constants.ArmConstants.ArmPosInValue), Commands.runOnce(() -> s_ShooterSubsystem.set(0.5, 0.40))));
        // ArmPosOut.onFalse(Stow());
        ArmPosOut.onTrue(new ArmPIDCommand(a_ArmSubsystem, Constants.ArmConstants.ArmPosOutValue));
        DeflectorPosIn.onTrue(DeflectorIn());
        DeflectorPosOut.onTrue(DeflectorOut());
        ShootS.onTrue(Commands.runOnce(() -> s_ShooterSubsystem.set(0.67, 0.47)));
        ShootS.onFalse(new SequentialCommandGroup(new WaitCommand(0.1), Outtake(), new WaitCommand(0.5), Commands.runOnce(() -> s_ShooterSubsystem.set(0, 0))));
        ShootA.onTrue(ShootACommand());
        onandstow.onTrue(OnAndStow());
        LiftPosOut.onTrue(new ParallelCommandGroup(new ClimberPIDCommand(c_ClimberSubsystem,
                Constants.ClimberConstants.LeftLiftPosInValue, Constants.ClimberConstants.RightLiftPosInValue),
                DeflectorOut()));
        LiftPosIn.onTrue(new ClimberPIDCommand(c_ClimberSubsystem, Constants.ClimberConstants.LeftLiftPosOutValue,
                Constants.ClimberConstants.RightLiftPosOutValue));

         AutoAim.whileTrue(new SequentialCommandGroup(
                 new InstantCommand(()-> l_LimelightSubsystem.setCamMode(0)), AutoPickUp(0)));

         AutoAim.onFalse(new ParallelCommandGroup(new InstantCommand(() -> FollowPID = 0),
                 new InstantCommand(() -> AimPID = 0), new InstantCommand(()-> l_LimelightSubsystem.setCamMode(0))));
         AutoShoot.whileTrue(new SequentialCommandGroup(
                 new ParallelCommandGroup(new AutoAimCommand(() -> l_LimelightBackSubsystem.getTargetX(),
                         () -> l_LimelightSubsystem.IsTargetAvailable()), Commands.runOnce(() -> s_ShooterSubsystem.set(0.52, 0.42))),
                 Outtake(), Commands.runOnce(() -> s_ShooterSubsystem.set(0, 0))));

         AutoShoot.onFalse(new ParallelCommandGroup(new InstantCommand(() -> AimPID = 0), Commands.runOnce(() -> s_ShooterSubsystem.set(0, 0))));

         //AutoAmp.onTrue(AutoAmpScore(l_LimelightBackSubsystem));
         //AutoAmp.onFalse(Commands.runOnce(() -> s_ShooterSubsystem.set(0, 0)));

        Up.whileTrue(new AutoRotateCommand(0, s_Swerve, 0, 0));
        Right.whileTrue(new AutoRotateCommand(270, s_Swerve, 0, 0));
        Down.whileTrue(new AutoRotateCommand(180, s_Swerve, 0, 0));
        Left.whileTrue(new AutoRotateCommand(90, s_Swerve, 0, 0));
        

        OutIntake.onTrue(ArmMove(-0.4));
        OutIntake.onFalse(ArmMove(0));
        InIntake.onTrue(ArmMove(0.4));
        InIntake.onFalse(ArmMove(0));
        InDeflector.onTrue(DeflectorMove(-0.5));
        InDeflector.onFalse(DeflectorMove(0));
        OutDeflector.onTrue(DeflectorMove(0.5));
        OutDeflector.onFalse(DeflectorMove(0));

        // increaseTopSpeed.onTrue(wrapSpeedChange(this::nextTopSpeed));
        // decreaseTopSpeed.onTrue(wrapSpeedChange(this::prevTopSpeed));
        // increaseBottomSpeed.onTrue(wrapSpeedChange(this::nextBottomSpeed));
        // decreaseBottomSpeed.onTrue(wrapSpeedChange(this::prevBottomSpeed));
    }

    void nextTopSpeed() {
        this.topShooterSpeed = this.topShooterSpeed.next();
    }

    void nextBottomSpeed() {
        this.bottomShooterSpeed = this.bottomShooterSpeed.next();
    }

    void prevTopSpeed() {
        this.topShooterSpeed = this.topShooterSpeed.prev();
    }

    void prevBottomSpeed() {
        this.bottomShooterSpeed = this.bottomShooterSpeed.prev();
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
