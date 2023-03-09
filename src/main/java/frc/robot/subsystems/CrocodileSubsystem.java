package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class CrocodileSubsystem extends SubsystemBase {
    private static final int FREE_CURRENT_LIMIT = 30;
    private static final int STALL_CURRENT_LIMIT = 20;

    private final Solenoid wristSolenoid;
    private final Solenoid chomperSolenoid;
    private final WPI_TalonFX intake;

    public CrocodileSubsystem() {
        intake = new WPI_TalonFX(RobotMap.Crocodile.INTAKE);
        intake.setNeutralMode(NeutralMode.Brake);
        /* enabled | Limit(amp) | Trigger Threshold(amp) | Trigger Threshold Time(s) */
        intake.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 35, 60, 0.1));

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
        // if (currentMotorPower > 0 && stallDebouncer.calculate((getMotorVelocity() <
        // 50))) {
        // setMotorSpeed(0);
        // }
    }
}
