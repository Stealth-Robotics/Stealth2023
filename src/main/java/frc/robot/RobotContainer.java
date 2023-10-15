package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.AutoIntakeCommand;
import frc.robot.commands.Autos.DO_NOTHING;
import frc.robot.commands.Autos.EXIT_COMMUNITY;
import frc.robot.commands.Autos.PreloadCubeExit;
import frc.robot.commands.Autos.PreloadCubeRight;
import frc.robot.commands.Autos.PreloadParkCenter;
import frc.robot.commands.Autos.PreloadPlusOneLeft;
import frc.robot.commands.Autos.PreloadPlusOneRight;
import frc.robot.commands.DefaultCommands.CrocodileDefaultCommand;
import frc.robot.commands.DefaultCommands.IntakeDefaultCommand;
import frc.robot.commands.DefaultCommands.RotatorDefaultCommand;
import frc.robot.commands.DefaultCommands.TeleopDrivebaseDefaultCommand;
import frc.robot.commands.DefaultCommands.TelescopeDefault;
import frc.robot.commands.Presets.HighPresetSequence;
import frc.robot.commands.Presets.MidPresetSequence;
import frc.robot.commands.Presets.PickupPresetSequence;
import frc.robot.commands.Presets.StowPresetSequence;
import frc.robot.commands.Presets.SubstationPickupPresetSequence;
import frc.robot.subsystems.CrocodileSubsystem;
import frc.robot.subsystems.Gamepiece;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.RotatorSubsystem;
import frc.robot.subsystems.TelescopeSubsystem;
import frc.robot.subsystems.Swerve.DrivebaseSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  public static final int k_DRIVER_CONTROLLER_PORT = 0;
  public static final int k_OPERATOR_CONTROLLER_PORT = 1;
  /* Controllers */
  private final Joystick driver = new Joystick(0);
  private final CommandXboxController driverController = new CommandXboxController(
      k_DRIVER_CONTROLLER_PORT);

  private final CommandXboxController mechController = new CommandXboxController(
      k_OPERATOR_CONTROLLER_PORT);

  /* Drive Controls */
  private final int translationAxis = XboxController.Axis.kLeftY.value;
  private final int strafeAxis = XboxController.Axis.kLeftX.value;
  private final int rotationAxis = XboxController.Axis.kRightX.value;

  /* Driver Buttons */
  private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kA.value);
  // private final JoystickButton robotCentric = new JoystickButton(driver,
  // XboxController.Button.kLeftBumper.value);

  /* Subsystems */
  private final DrivebaseSubsystem swerve;
  private final RotatorSubsystem rotator;
  private final CrocodileSubsystem endEffector;
  private final TelescopeSubsystem telescope;
  private final IntakeSubsystem intake;
  //private final CandleSubsystem candle;

  private UsbCamera camera = CameraServer.startAutomaticCapture();
  private SendableChooser<Command> autoChooser = new SendableChooser<>();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    swerve = new DrivebaseSubsystem();
    telescope = new TelescopeSubsystem();
    SmartDashboard.putData("Telescope", telescope);
    rotator = new RotatorSubsystem(telescope);
    SmartDashboard.putData("Rotator", rotator);
    endEffector = new CrocodileSubsystem();
    SmartDashboard.putData("Intake", endEffector);
    intake = new IntakeSubsystem();
    //candle = new CandleSubsystem();

    camera.setResolution(160, 120);
    camera.setFPS(30);

    swerve.setDefaultCommand(
        new TeleopDrivebaseDefaultCommand(
            swerve,
            () -> -driverController.getRawAxis(translationAxis),
            () -> -driverController.getRawAxis(strafeAxis),
            () -> -driverController.getRawAxis(rotationAxis),
            () -> driverController.b().getAsBoolean(),
            () -> driverController.rightBumper().getAsBoolean()));

    rotator.setDefaultCommand(new RotatorDefaultCommand(
        rotator,
        telescope,
        () -> -mechController.getRightY()));

    telescope.setDefaultCommand(
        new TelescopeDefault(
            telescope,
            () -> -mechController.getLeftY(),
            mechController.leftBumper()));

    endEffector.setDefaultCommand(new CrocodileDefaultCommand(
        endEffector,
        () -> (mechController.getRightTriggerAxis() - mechController.getLeftTriggerAxis())));

    intake.setDefaultCommand(new IntakeDefaultCommand(
      intake,
      () -> (driverController.getRightTriggerAxis() - driverController.getLeftTriggerAxis())
    ));

    autoChooser.setDefaultOption("CENTER Preload Park", new PreloadParkCenter(swerve, endEffector, rotator, telescope, intake));
    autoChooser.addOption("RIGHT Preload + 1", new PreloadPlusOneRight(swerve, endEffector, rotator, telescope, intake));
    autoChooser.addOption("LEFT Preload + 1", new PreloadPlusOneLeft(swerve, endEffector, rotator, telescope, intake));
    autoChooser.addOption("EXIT COMMUNITY", new EXIT_COMMUNITY(swerve, endEffector, rotator, telescope));
    autoChooser.addOption("DO NOTHING", new DO_NOTHING(swerve, endEffector, rotator, telescope));
    autoChooser.addOption("preload cube exit", new PreloadCubeExit(swerve, endEffector, rotator, telescope, intake));
    autoChooser.addOption("preloadright", new PreloadCubeRight(swerve, endEffector, rotator, telescope, intake));
    SmartDashboard.putData("Selected Autonomous", autoChooser);

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

    zeroGyro.onTrue(new InstantCommand(() -> swerve.zeroGyro()));

    mechController.a().onTrue(new PickupPresetSequence(telescope, rotator, endEffector, intake, driverController.y()));
    mechController.y()
        .onTrue(new StowPresetSequence(telescope, rotator, endEffector, intake,
            () -> (driverController.getRightTriggerAxis() - driverController.getLeftTriggerAxis()),
            () -> endEffector.getGamePiece()));
    mechController.x().onTrue(new HighPresetSequence(telescope, rotator, endEffector, intake,
        () -> (driverController.getRightTriggerAxis() - driverController.getLeftTriggerAxis()),
        () -> endEffector.getGamePiece()));
    mechController.b().onTrue(new MidPresetSequence(telescope, rotator, endEffector, intake,
        () -> (driverController.getRightTriggerAxis() - driverController.getLeftTriggerAxis()),
        () -> endEffector.getGamePiece()));
    mechController.povLeft().onTrue(new InstantCommand(() -> telescope.resetEncoder()));
    mechController.button(8).onTrue(new InstantCommand(() -> {
      endEffector.setGamePiece(Gamepiece.CONE);
      intake.setGamePiece(Gamepiece.CONE);
      //candle.cone();
    }, endEffector));
    mechController.button(7).onTrue(new InstantCommand(() -> {
      endEffector.setGamePiece(Gamepiece.CUBE);
      intake.setGamePiece(Gamepiece.CUBE);
      //candle.cube();
    }, endEffector));
    mechController.povDown()
        .onTrue(new SubstationPickupPresetSequence(telescope, rotator, endEffector, intake, driverController.y(),
            () -> endEffector.getGamePiece()));
    // driverController.y().onTrue(new AutoIntakeCommand(endEffector, 0.5, driverController.y()));
    driverController.leftBumper().onTrue(new AutoIntakeCommand(intake, 0.75, driverController.leftBumper()));
  }

  public void teleopInit() {
    //telescope.onInit();
    rotator.onInit();
    endEffector.onInit();
  }

  public void autonomousInit() {
    //telescope.onInit();
    rotator.onInit();
    endEffector.onInit();
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // System.out.println("Selected Autonomous: " + autoChooser.getSelected());
    return autoChooser.getSelected();
    // return null;
    // return new InstantCommand();
    // return new BluePreloadOnly(swerve, endEffector, rotator, telescope);
  }

}
