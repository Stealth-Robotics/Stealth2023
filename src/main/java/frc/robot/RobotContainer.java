package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.RotatorSubsystem;
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
  private final TelescopeSubsystem telescope;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    swerve = new DrivebaseSubsystem();
    telescope = new TelescopeSubsystem();
    rotator = new RotatorSubsystem();

    swerve.setDefaultCommand(
        new TeleopDrivebaseDefaultCommand(
            swerve,
            () -> -driverController.getRawAxis(translationAxis),
            () -> -driverController.getRawAxis(strafeAxis),
            () -> -driverController.getRawAxis(rotationAxis),
            () -> driverController.b().getAsBoolean() // ,
        // () -> driverController.leftBumper().getAsBoolean()
        ));

    rotator.setDefaultCommand(new RotatorDefaultCommand(
        rotator,
        () -> -mechController.getRightX()));
    
    telescope.setDefaultCommand(
      
        new TelescopeDefault(
            telescope,
            () -> mechController.getLeftX()));

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
    mechController.b().onTrue(new ResetTelescope(telescope));
    zeroGyro.onTrue(new InstantCommand(() -> swerve.zeroGyro()));

    mechController
        .x()
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
    return new BluePreloadPlusOneLeft(swerve);
  }
}
