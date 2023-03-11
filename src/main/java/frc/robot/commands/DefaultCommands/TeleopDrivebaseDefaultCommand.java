package frc.robot.commands.DefaultCommands;

import frc.robot.SharedConstants;
import frc.robot.subsystems.Swerve.DrivebaseSubsystem;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class TeleopDrivebaseDefaultCommand extends CommandBase {
    private DrivebaseSubsystem drivebase;
    private DoubleSupplier translationSup;
    private DoubleSupplier strafeSup;
    private DoubleSupplier rotationSup;
    private BooleanSupplier robotCentricSup;
    private BooleanSupplier slowMode;

    private final double SLOW_MODE_MULTIPLIER = 0.5;
    public static final double SITCK_DEADBAND = 0.1;

    public TeleopDrivebaseDefaultCommand(DrivebaseSubsystem s_Swerve, DoubleSupplier translationSup,
            DoubleSupplier strafeSup, DoubleSupplier rotationSup, BooleanSupplier robotCentricSup,
            BooleanSupplier slowMode) {
        this.drivebase = s_Swerve;

        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
        this.rotationSup = rotationSup;
        this.robotCentricSup = robotCentricSup;

        addRequirements(s_Swerve);

        this.slowMode = slowMode;
    }

    @Override
    public void execute() {
        /* Get Values, Deadband */
        double translationVal = MathUtil.applyDeadband(translationSup.getAsDouble(), SITCK_DEADBAND);
        double strafeVal = MathUtil.applyDeadband(strafeSup.getAsDouble(), SITCK_DEADBAND);
        double rotationVal = MathUtil.applyDeadband(rotationSup.getAsDouble(), SITCK_DEADBAND);
        if (slowMode.getAsBoolean()) {
            translationVal *= SLOW_MODE_MULTIPLIER;
            strafeVal *= SLOW_MODE_MULTIPLIER;
            rotationVal *= SLOW_MODE_MULTIPLIER;
        }
        /* Drive */
        drivebase.drive(
                new Translation2d(translationVal, strafeVal).times(SharedConstants.DrivebaseConstants.MAX_SPEED),
                rotationVal * SharedConstants.DrivebaseConstants.MAX_ANGULAR_VELOCITY,
                !robotCentricSup.getAsBoolean(),
                true);
    }
}