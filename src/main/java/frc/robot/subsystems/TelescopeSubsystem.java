package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class TelescopeSubsystem extends SubsystemBase {
    // PID Constants
    // P for telescope PID
    private static final double P_COEFF = 0.1;
    // I for telescope PID
    private static final double I_COEFF = 0.0;
    // D for telescope PID
    private static final double D_COEFF = 0.025;
    // Tolerance for telescope PID
    private static final double POSITIONAL_TOLERANCE = 1000;
    // Velocity tolerance for telescope PID
    private static final double VELOCITY_TOLERANCE = Double.POSITIVE_INFINITY;
    // F for telescope PID
    private static final double F_COEFF = 0;
    // The telecope cannot exceed these ticks
    private static final int UPPER_BOUND = 97105;

    // Telescope motor
    private final WPI_TalonFX telescopeMotor;
    // Debouncer to measure when the telescope is retracted and the motor is
    // stalling
    // We landed on 0.050 seconds as the debounce time because it was the good
    // middle ground between false positives and fase negatives
    private Debouncer stallDebouncer = new Debouncer(0.050, Debouncer.DebounceType.kRising);
    // The current setpoint of the telescope
    private double currentSetpoint;
    // The telescope cannot exceed these ticks
    private final int MAXIMUM_TICKS = -75000;

    public TelescopeSubsystem() {
        // Config motor settings
        telescopeMotor = new WPI_TalonFX(RobotMap.Telescope.TELESCOPE_ID);
        telescopeMotor.configFactoryDefault();
        telescopeMotor.setNeutralMode(NeutralMode.Brake);
        telescopeMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        telescopeMotor.configAllowableClosedloopError(0, POSITIONAL_TOLERANCE);
        // Config motor PID values
        telescopeMotor.config_kP(0, P_COEFF);
        telescopeMotor.config_kI(0, I_COEFF);
        telescopeMotor.config_kD(0, D_COEFF);
        telescopeMotor.config_kF(0, F_COEFF);
    }

    // Gets the setpoint of the internal PID
    private double getSetpoint() {
        return currentSetpoint;
    }

    // Gets the setpoint of the internal PID in a percentage of the maximum
    // extension
    public double getSetpointPercent() {
        return getSetpoint() / MAXIMUM_TICKS;
    }

    // Gets the current position of the telescope in a percentage of the maximum
    // extension
    public double getExtensionPercent() {
        return getCurrentPosition() / MAXIMUM_TICKS;
    }

    // Sets the PID setpoint to the current position
    public void setToCurrentPosition() {
        setSetpoint(getCurrentPosition());
    }

    // Gets the current position of the telescope in encoder ticks
    private double getCurrentPosition() {
        return telescopeMotor.getSelectedSensorPosition();
    }

    // Rests the encoder to 0
    public void resetEncoder() {
        telescopeMotor.setSelectedSensorPosition(0);
    }

    // Converts a percentage of extension to encoder ticks
    private double percentToTicks(double percent) {
        return percent * MAXIMUM_TICKS;
    }

    // Sets the speed of the telescope
    // This should only ever be used for manual control
    public void setSpeed(double speed) {
        currentSetpoint = getCurrentPosition();
        telescopeMotor.set(ControlMode.PercentOutput, speed);
    }

    // Tells the PID where to go (percentage of extension)
    public void setExtensionPercent(double percentExtension) {
        setSetpoint(percentToTicks(percentExtension));
    }

    // Tells the PID where to go
    private void setSetpoint(double positionTicks) {
        telescopeMotor.set(ControlMode.Position, positionTicks);
        currentSetpoint = positionTicks;
    }

    // Returns true if the motor is stalling, false otherwise
    public boolean checkVelocity() {
        return stallDebouncer.calculate(
                Math.abs(telescopeMotor.getSelectedSensorVelocity()) < 50);
    }

    // Resets the telescope encoder and setpoint
    public void completeReset() {
        telescopeMotor.set(ControlMode.PercentOutput, 0);
        telescopeMotor.setSelectedSensorPosition(100);// 100 is 100 ticks backwards
        setSetpoint(0);
    }

    // Move the telescope down
    // This is used for resetting the telescope
    public void retractTelescope() {
        telescopeMotor.set(ControlMode.PercentOutput, -0.3);
        stallDebouncer.calculate(false);
    }

    // Returns true if the telescope is at its setpoint, false otherwise
    public boolean atSetpoint() {
        return Math.abs(currentSetpoint - getCurrentPosition()) < POSITIONAL_TOLERANCE;
    }

    // Returns true if the telescope is within its upper bound, false otherwise
    public boolean inBounds() {
        return getExtensionPercent() < 1;
    }

    @Override
    public void periodic() {
        System.out.println(getCurrentPosition() +  " | err:" +  Math.abs(telescopeMotor.getClosedLoopError()));
    }
}
