package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotMap;

public class TelescopeSubsystem extends SubsystemBase {
    private final WPI_TalonFX telescopeMotor;
    private Debouncer stallDebouncer = new Debouncer(0.050, Debouncer.DebounceType.kRising);

    private double speedLimit;

    private double currentSetpoint;
    private final int MAXIMUM_TICKS = -1; //TODO: set to actual value
    public TelescopeSubsystem() {

        telescopeMotor = new WPI_TalonFX(RobotMap.Telescope.TELESCOPE_ID);

        telescopeMotor.configFactoryDefault();

        telescopeMotor.setNeutralMode(NeutralMode.Brake);

        telescopeMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);

        telescopeMotor.configAllowableClosedloopError(0, Constants.TelescopeConstants.POSITIONAL_TOLERANCE);

        telescopeMotor.config_kP(0, Constants.TelescopeConstants.P_COEFF);
        telescopeMotor.config_kI(0, Constants.TelescopeConstants.I_COEFF);
        telescopeMotor.config_kD(0, Constants.TelescopeConstants.D_COEFF);
        telescopeMotor.config_kF(0, Constants.TelescopeConstants.F_COEFF);

        telescopeMotor.setSelectedSensorPosition(0);

        // we dont need to invert the motor and neither properly invert the sensor
        // We have decided to just cope with negative numbers
        // telescopeMotor.setSensorPhase(true);
        // telescopeMotor.setInverted(true);

        currentSetpoint = getCurrentPosition();
    }

    public void setSetpoint(double setpoint) {
        this.currentSetpoint = setpoint;
    }

    public double getSetpoint() {
        return currentSetpoint;
    }

    public double currentTicksToPercent(){
        return getCurrentPosition() / MAXIMUM_TICKS;
    }
    public void resetEncoder(){
        telescopeMotor.setSelectedSensorPosition(0);
    }
    public double percentToTicks(double percent){
        return percent * MAXIMUM_TICKS;
    }

    public double ticksToPercent(double ticks){
        return ticks / MAXIMUM_TICKS;
    }

    public double getCurrentPosition() {
        return telescopeMotor.getSelectedSensorPosition();
    }

    public void setSpeed(double speed) {
        currentSetpoint = getCurrentPosition();
        telescopeMotor.set(ControlMode.PercentOutput, speed);
    }

    public void setPosition(double positionTicks) {
        telescopeMotor.set(ControlMode.Position, positionTicks);
    }

    public boolean checkVelocity() {

        return stallDebouncer.calculate(
                Math.abs(telescopeMotor.getSelectedSensorVelocity()) < 50);
    }

    public void completeReset() {
        telescopeMotor.set(ControlMode.PercentOutput, 0);
        telescopeMotor.setSelectedSensorPosition(100);//100 is 100 ticks backwards
        setSetpoint(0);
    }

    // check the elevator down
    public void retractTelescope() {
        telescopeMotor.set(ControlMode.PercentOutput, 0.3);
        stallDebouncer.calculate(false);
    }

    public boolean atSetpoint() {
        return Math.abs(telescopeMotor.getClosedLoopError()) < Constants.TelescopeConstants.POSITIONAL_TOLERANCE
                && Math.abs(telescopeMotor.getSelectedSensorVelocity()) < Constants.TelescopeConstants.VELOCITY_TOLERANCE;
    }

    public boolean inBounds(){
        return true;
        //return Math.abs(getCurrentPosition()) <= Constants.TelescopeConstants.UPPER_BOUND ;
    }

    @Override
    public void periodic() {
        //System.out.println(armMotor.getSelectedSensorVelocity());
        //System.out.println(getCurrentPosition());
    }
}
