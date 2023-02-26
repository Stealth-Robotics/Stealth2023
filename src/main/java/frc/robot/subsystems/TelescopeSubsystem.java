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
    private final WPI_TalonFX armMotor;
    private Debouncer stallDebouncer = new Debouncer(0.010, Debouncer.DebounceType.kRising);

    private double currentSetpoint;

    public TelescopeSubsystem() {

        armMotor = new WPI_TalonFX(RobotMap.Telescope.TELESCOPE_ID);

        armMotor.configFactoryDefault();

        armMotor.setNeutralMode(NeutralMode.Brake);

        armMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);

        armMotor.configAllowableClosedloopError(0, Constants.TelescopeConstants.POSITIONAL_TOLERANCE);

        armMotor.config_kP(0, Constants.TelescopeConstants.P_COEFF);
        armMotor.config_kI(0, Constants.TelescopeConstants.I_COEFF);
        armMotor.config_kD(0, Constants.TelescopeConstants.D_COEFF);
        armMotor.config_kF(0, Constants.TelescopeConstants.F_COEFF);

        armMotor.setSelectedSensorPosition(0);

        currentSetpoint = getCurrentPosition();
        register();
    }

    public void setSetpoint(double setpoint) {
        this.currentSetpoint = setpoint;
    }

    public double getSetpoint() {
        return currentSetpoint;
    }

    public double getCurrentPosition() {
        return armMotor.getSelectedSensorPosition();
    }

    public void setSpeed(double speed) {
        currentSetpoint = getCurrentPosition();
        armMotor.set(ControlMode.PercentOutput, speed);
    }

    public void setPosition(double position) {
        armMotor.set(ControlMode.Position, position);
    }

    public boolean checkVelocity() {

        return stallDebouncer.calculate(
                Math.abs(armMotor.getSelectedSensorVelocity()) < 50);
    }

    public void completeReset() {
        armMotor.set(ControlMode.PercentOutput, 0);
        armMotor.setSelectedSensorPosition(0);
    }

    // check the elevator down
    public void retractTelescope() {
        armMotor.set(ControlMode.PercentOutput, 0.3);
        stallDebouncer.calculate(false);
    }

    public boolean atSetpoint() {
        return Math.abs(armMotor.getClosedLoopError()) < Constants.TelescopeConstants.POSITIONAL_TOLERANCE
                && Math.abs(armMotor.getSelectedSensorVelocity()) < Constants.TelescopeConstants.VELOCITY_TOLERANCE;
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
