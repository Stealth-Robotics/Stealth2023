package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class CrocodileSubsystem extends SubsystemBase {
    private final Solenoid wristSolenoid;
    private final Solenoid chomperSolenoid;
    private final WPI_TalonFX intake;
    private Debouncer stallDebouncer = new Debouncer(0.050, Debouncer.DebounceType.kRising);
    private double currentMotorPower;

    public CrocodileSubsystem() {
        intake = new WPI_TalonFX(RobotMap.Crocodile.INTAKE);
        intake.setNeutralMode(NeutralMode.Brake);
        // intake.setSmartCurrentLimit(Constants.CrocodileConstants.STALL_CURRENT_LIMIT,
        //         Constants.CrocodileConstants.FREE_CURRENT_LIMIT);
        // intake.burnFlash();

        wristSolenoid = new Solenoid(
                RobotMap.Pneumatics.PCM,
                RobotMap.Pneumatics.PCM_TYPE,
                RobotMap.Pneumatics.CLAW_PCM_CHANNEL);
        chomperSolenoid = new Solenoid(
                RobotMap.Pneumatics.PCM,
                RobotMap.Pneumatics.PCM_TYPE,
                RobotMap.Pneumatics.CHOMPER_PCM_CHANNEL);
    }

    public void setMotorSpeed(double speed) {
        intake.set(speed);
        currentMotorPower = speed;
    }

    public double getMotorVelocity() {
        return intake.getSelectedSensorVelocity();
    }

    private void setWrist(boolean newValue) {
        wristSolenoid.set(newValue);
    }

    public void toggleWrist() {
        wristSolenoid.toggle();
    }

    public boolean getWristPositionBool() {
        return wristSolenoid.get();
    }

    public void wristUp() {
        setWrist(true);
    }

    public void wristDown() {
        setWrist(false);
    }

    private void setChomper(boolean newValue) {
        chomperSolenoid.set(newValue);
    }

    public void toggleChomper() {
        chomperSolenoid.toggle();
    }

    public void closeChomper() {
        setChomper(true);
    }

    public void openChomper() {
        setChomper(false);
    }

    public boolean getChomperPositionBool() {
        return chomperSolenoid.get();
    }

    @Override
    public void periodic() {
        if (currentMotorPower > 0 && stallDebouncer.calculate((getMotorVelocity() < 50))) {
            setMotorSpeed(0);
        }
    }
}
