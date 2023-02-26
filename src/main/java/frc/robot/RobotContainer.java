package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.RotatorDefaultCommand;
import frc.robot.commands.TeleopDrivebaseDefaultCommand;
import frc.robot.subsystems.RotatorSubsystem;
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

  /* Drive Controls */
  private final int translationAxis = XboxController.Axis.kLeftY.value;
  private final int strafeAxis = XboxController.Axis.kLeftX.value;
  private final int rotationAxis = XboxController.Axis.kRightX.value;

  /* Driver Buttons */
  private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kY.value);
  private final JoystickButton robotCentric = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);

  /* Subsystems */
  private final DrivebaseSubsystem s_Swerve = new DrivebaseSubsystem();
  private final RotatorSubsystem rotator;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    rotator = new RotatorSubsystem();
    rotator.setDefaultCommand(new RotatorDefaultCommand(
        rotator,
        () -> -driverController.getRightX()));

    s_Swerve.setDefaultCommand(
        new TeleopDrivebaseDefaultCommand(
            s_Swerve,
            () -> -driverController.getRawAxis(translationAxis),
            () -> -driverController.getRawAxis(strafeAxis),
            () -> -driverController.getRawAxis(rotationAxis),
            () -> driverController.b().getAsBoolean()// ,
        // () -> driverController.leftBumper().getAsBoolean()
        // TODO: Uncomment When Other Half Of This Commit Comes in Through LevelRobotPR
        ));

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link
   * CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureButtonBindings() {
    /* Driver Buttons */
    zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));

    driverController
        .a()
        .onTrue(
            Commands.runOnce(
                () -> {
                  rotator.setGoal(130);
                },
                rotator));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // TODO: Replace with Auto command
    return Commands.none();
  }
}
