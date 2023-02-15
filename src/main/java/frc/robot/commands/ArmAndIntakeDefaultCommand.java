package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.Intake;

public class ArmAndIntakeDefaultCommand extends CommandBase {
    private Intake intake;
    private ArmSubsystem arm;
    private DoubleSupplier intakeDoubleSupplier, armDoubleSupplier;
    private PIDController pid;
    private long endLoop;

    public ArmAndIntakeDefaultCommand(
            Intake intake,
            ArmSubsystem arm,
            DoubleSupplier intakeDoubleSupplier,
            DoubleSupplier armDoubleSupplier) {
        this.intake = intake;
        this.arm = arm;
        this.intakeDoubleSupplier = intakeDoubleSupplier;
        this.armDoubleSupplier = armDoubleSupplier;
        pid = new PIDController(0, 0, 0);
        addRequirements(intake, arm);
    }

    @Override
    public void initialize() {
        endLoop = System.nanoTime() / (long) Math.pow(10, 9);
    }

    @Override
    public void execute() {
        long temp = endLoop;
        endLoop = System.nanoTime() / (long) Math.pow(10, 9);
        long deltaTime = endLoop - temp;
        intake.setIntakePower(intakeDoubleSupplier.getAsDouble());
        double setpoint = MathUtil.clamp(pid.getSetpoint() + (armDoubleSupplier.getAsDouble() * deltaTime * Constants.ArmConstants.TICKS_PER_SECOND), ArmConstants.LOWER_BOUND_INTAKE_OUT_TICKS, ArmConstants.UPPER_BOUND_TICKS);
        if ((setpoint < ArmConstants.LOWER_BOUND_INTAKE_IN_TICKS) && !intake.isIntakeOut()) {
            CommandScheduler.getInstance().schedule(new IntakeOutUpdateSetpoint(intake, pid, deltaTime));
        }
        else{
            pid.setSetpoint(setpoint);
        }
        arm.setMotorPower(pid.calculate(arm.getEncoderValue()));
        
        if(arm.getEncoderValue() > Constants.ArmConstants.LOWER_BOUND_INTAKE_IN_TICKS){
            intake.retractIntake();
        }
    }
}
