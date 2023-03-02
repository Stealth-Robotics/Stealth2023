package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotMap;

public class CrocodileSubsystem extends SubsystemBase {
    private final Solenoid wristSolenoid;
    private final Solenoid chomperSolenoid;
    private final CANSparkMax intake;
    private Debouncer stallDebouncer = new Debouncer(0.050, Debouncer.DebounceType.kRising);
    private double currentMotorPower;

    public CrocodileSubsystem() {
        intake = new CANSparkMax(RobotMap.Crocodile.INTAKE, MotorType.kBrushless);
        intake.setSmartCurrentLimit(Constants.CrocodileConstants.STALL_CURRENT_LIMIT,
                Constants.CrocodileConstants.FREE_CURRENT_LIMIT);
        intake.burnFlash();

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
        return intake.getEncoder().getVelocity();
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
