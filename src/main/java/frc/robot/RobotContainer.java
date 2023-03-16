package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.RotatorSubsystem;
import frc.robot.subsystems.CrocodileSubsystem;
import frc.robot.commands.*;
import frc.robot.commands.Autos.BluePreloadOnly;
import frc.robot.commands.Autos.BluePreloadParkCenter;
import frc.robot.commands.Autos.RedPreloadOnly;
import frc.robot.commands.Autos.RedPreloadParkCenter;
import frc.robot.commands.DefaultCommands.CrocodileDefaultCommand;
import frc.robot.commands.DefaultCommands.RotatorDefaultCommand;
import frc.robot.commands.DefaultCommands.TeleopDrivebaseDefaultCommand;
import frc.robot.commands.DefaultCommands.TelescopeDefault;
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
  // private final JoystickButton robotCentric = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);

  /* Subsystems */
  private final DrivebaseSubsystem swerve;
  private final RotatorSubsystem rotator;
  private final CrocodileSubsystem endEffector;
  private final TelescopeSubsystem telescope;

  private UsbCamera camera = CameraServer.startAutomaticCapture();
  private SendableChooser<Command> autoChooser = new SendableChooser<>();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    swerve = new DrivebaseSubsystem();
    telescope = new TelescopeSubsystem();
    rotator = new RotatorSubsystem();
    endEffector = new CrocodileSubsystem();

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
            () -> -mechController.getLeftY()));

    autoChooser.setDefaultOption("Blue 1+Park", new BluePreloadParkCenter(swerve, endEffector, rotator, telescope));
    autoChooser.addOption("Blue Preload", new BluePreloadOnly(swerve, endEffector, rotator, telescope));
    autoChooser.addOption("Red 1+Park", new RedPreloadParkCenter(swerve, endEffector, rotator, telescope));
    autoChooser.addOption("Red Preload", new RedPreloadOnly(swerve, endEffector, rotator, telescope));

    SmartDashboard.putData("Selected Autonomous", autoChooser);
    endEffector.setDefaultCommand(new CrocodileDefaultCommand(
        endEffector,
        () -> (driverController.getRightTriggerAxis() - driverController.getLeftTriggerAxis()),
        (t) -> driverController.getHID().setRumble(RumbleType.kBothRumble, t)));

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

    mechController.a().onTrue(new RotatorToPosition(rotator, telescope, 230));
    mechController.y().onTrue(new RotatorToPosition(rotator, telescope, 40));
    mechController.x().onTrue(new TelescopeToPosition(telescope, 0.9));
    mechController.b().onTrue(new InstantCommand(() -> telescope.resetEncoder()));
  }

  public void teleopInit() {
    telescope.setToCurrentPosition();
  }

  public void autonomousInit() {
    telescope.setToCurrentPosition();
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // System.out.println("Selected Autonomous: " + autoChooser.getSelected());
    // return autoChooser.getSelected();
    // return null;
    return new BluePreloadOnly(swerve, endEffector, rotator, telescope);
  }
}
