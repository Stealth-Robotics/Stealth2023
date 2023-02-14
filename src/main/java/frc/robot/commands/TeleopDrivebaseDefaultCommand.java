package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.Swerve.DrivebaseSubsystem;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;


public class TeleopDrivebaseDefaultCommand extends CommandBase {    
    private DrivebaseSubsystem drivebase;    
    private DoubleSupplier translationSup;
    private DoubleSupplier strafeSup;
    private DoubleSupplier rotationSup;
    private BooleanSupplier robotCentricSup;
    private BooleanSupplier slowModeSupplier;

    public TeleopDrivebaseDefaultCommand(DrivebaseSubsystem s_Swerve, DoubleSupplier translationSup, DoubleSupplier strafeSup, DoubleSupplier rotationSup, BooleanSupplier robotCentricSup, BooleanSupplier slowModeSupplier) {
        this.drivebase = s_Swerve;

        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
        this.rotationSup = rotationSup;
        this.robotCentricSup = robotCentricSup;
        this.slowModeSupplier = slowModeSupplier;

        addRequirements(s_Swerve);

    }

    @Override
    public void execute() {
        /* Get Values, Deadband*/
        double translationVal = MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.TeleopConstants.SITCK_DEADBAND);
        double strafeVal = MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.TeleopConstants.SITCK_DEADBAND);
        double rotationVal = MathUtil.applyDeadband(rotationSup.getAsDouble(), Constants.TeleopConstants.SITCK_DEADBAND);

        if (slowModeSupplier.getAsBoolean()) {
            translationVal *= Constants.DrivebaseConstants.slowmodeMultiplier;
            strafeVal *= Constants.DrivebaseConstants.slowmodeMultiplier;
            rotationVal *= Constants.DrivebaseConstants.slowmodeMultiplier;
        }
        /* Drive */
        drivebase.drive(
            new Translation2d(translationVal, strafeVal).times(Constants.DrivebaseConstants.MAX_SPEED), 
            rotationVal * Constants.DrivebaseConstants.MAX_ANGULAR_VELOCITY, 
            !robotCentricSup.getAsBoolean(), 
            true
        );
    }
}