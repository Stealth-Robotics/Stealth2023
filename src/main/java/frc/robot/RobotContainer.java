package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.RotatorSubsystem;
import frc.robot.subsystems.CrocodileSubsystem;
import frc.robot.RobotMap.Crocodile;
import frc.robot.commands.*;
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
  /* Controllers */
  private final Joystick driver = new Joystick(0);
  private final CommandXboxController driverController = new CommandXboxController(
      Constants.IOConstants.k_DRIVER_CONTROLLER_PORT);

  private final CommandXboxController mechController = new CommandXboxController(
      Constants.IOConstants.k_OPERATOR_CONTROLLER_PORT);

  /* Drive Controls */
  private final int translationAxis = XboxController.Axis.kLeftY.value;
  private final int strafeAxis = XboxController.Axis.kLeftX.value;
  private final int rotationAxis = XboxController.Axis.kRightX.value;

  /* Driver Buttons */
  private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kA.value);
  private final JoystickButton robotCentric = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);

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
            () -> driverController.rightBumper().getAsBoolean()
        ));
    
    rotator.setDefaultCommand(new RotatorDefaultCommand(
        rotator,
        telescope,
        () -> -mechController.getRightY()));
    
    telescope.setDefaultCommand(      
        new TelescopeDefault(
            telescope,
            () -> -mechController.getLeftY()));

    // autoChooser.setDefaultOption("Blue 1+Park", new BluePreloadParkCenter(swerve));
    // autoChooser.addOption("Blue 1+1 Left", new BluePreloadPlusOneLeft(swerve));
    // autoChooser.addOption("Blue 1+1 Right", new BluePreloadPlusOneRight(swerve));
    //SmartDashboard.putData("Selected Autonomous", autoChooser);
    endEffector.setDefaultCommand(new CrocodileDefaultCommand(
      endEffector,
        () -> (driverController.getRightTriggerAxis() - driverController.getLeftTriggerAxis()),
        () -> driverController.leftBumper().getAsBoolean()));

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

    driverController.leftBumper().onTrue(new InstantCommand(() -> endEffector.toggleChomper(), endEffector));


    mechController.a().onTrue(new TelescopeToPosition(telescope, 50));
    mechController.y().onTrue(new RotatorToPosition(rotator, telescope, 90));
    mechController.x().onTrue(new RotatorToPosition(rotator, telescope, 230));
    mechController.b().onTrue(new InstantCommand(()->telescope.resetEncoder()));
    mechController.rightBumper().onTrue(new InstantCommand(() -> endEffector.toggleWrist(), endEffector));


    driverController.rightBumper().onTrue(new LevelRobot(swerve));
    // mechController
    // .x()
    // .onTrue(
    // Commands.runOnce(
    // () -> {
    // rotator.setGoal(130);
    // },
    // rotator));
  }

  public void teleopInit() {
    //telescope.completeReset();
    //telescope.resetEncoder();
    telescope.setSetpoint(telescope.getCurrentPosition());
  }

  public void autonomousInit() {
    //telescope.completeReset();
    telescope.setSetpoint(telescope.getCurrentPosition());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    //System.out.println("Selected Autonomous: " + autoChooser.getSelected());
    //return autoChooser.getSelected();
    return null;
  }
}
