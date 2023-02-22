package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Intake extends SubsystemBase {
    private final Solenoid intakeSolenoid;
    private final WPI_TalonFX rightIntakeMotor;
    private final WPI_TalonFX leftIntakeMotor;
    private final CANSparkMax rollerMotor;

    public Intake() {
        // Allowing the PCM to find the intake solenoid and what type it is.
        intakeSolenoid = new Solenoid(
                RobotMap.Pneumatics.PCM,
                RobotMap.Pneumatics.PCM_TYPE,
                RobotMap.Pneumatics.INTAKE_DEPLOY_PCM_CHANNEL);

        // Finding the motors for right and left.
        rightIntakeMotor = new WPI_TalonFX(RobotMap.IntakeIDs.RIGHT_INTAKE_MOTOR_ID);
        leftIntakeMotor = new WPI_TalonFX(RobotMap.IntakeIDs.LEFT_INTAKE_MOTOR_ID);

        // Sets the mode to brake when the motor is not powered.
        rightIntakeMotor.setNeutralMode(NeutralMode.Brake);
        leftIntakeMotor.setNeutralMode(NeutralMode.Brake);

        // Inverts the right motor to rotate clockwise.The left motor turns
        // counter-clockwise
        // by default, so this should pull the item into the intake.
        rightIntakeMotor.setInverted(true);

        // Makes the left motor follow or move when the right changes power.
        leftIntakeMotor.follow(rightIntakeMotor);

        // sets horizontal roller, neo 550 so we are using spark max motor controller
        rollerMotor = new CANSparkMax(RobotMap.IntakeIDs.ROLLER_MOTOR_ID, MotorType.kBrushless);
        rollerMotor.setIdleMode(IdleMode.kBrake);

        // Set the delay that sets of data are transmitted.
        leftIntakeMotor.setStatusFramePeriod(1, 255);
        leftIntakeMotor.setStatusFramePeriod(2, 255);
        rightIntakeMotor.setStatusFramePeriod(1, 255);
        rightIntakeMotor.setStatusFramePeriod(2, 255);
    }

    public void deployIntake() {
        intakeSolenoid.set(true);

    }

    public void retractIntake() {
        intakeSolenoid.set(false);

    }

    public void toggleIntake() {
        intakeSolenoid.toggle();

    }

    public void setIntakePower(double motorPower, double rollerPower) {
        rightIntakeMotor.set(ControlMode.PercentOutput, motorPower);
        rollerMotor.set(rollerPower);
    }

    public boolean isIntakeOut() {
        return intakeSolenoid.get();
    }
}
