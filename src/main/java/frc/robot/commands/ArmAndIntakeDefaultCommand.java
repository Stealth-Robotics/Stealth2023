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
        //gets loop time for consistent arm movement
        long deltaTime = endLoop - temp;
        //sets intake depending on speed and extends and retracts
        if(intakeDoubleSupplier.getAsDouble() > 0.05){
            intake.deployIntake();
            intake.setIntakePower(intakeDoubleSupplier.getAsDouble());
        }
        else{
            intake.retractIntake();
        }
        //sets setpoint depending on arm joystick, deltatime, and ticks per second constant, clamped between lower and upper bound
        double setpoint = MathUtil.clamp(pid.getSetpoint() + (armDoubleSupplier.getAsDouble() * deltaTime * Constants.ArmConstants.TICKS_PER_SECOND), ArmConstants.LOWER_BOUND_INTAKE_OUT_TICKS, ArmConstants.UPPER_BOUND_TICKS);
        //checks if setpoint will kill the robot, if so, schedules command that will deploy intake and update setpoint after half a second to make sure we don't kill the robot
        if ((setpoint < ArmConstants.LOWER_BOUND_INTAKE_IN_TICKS) && !intake.isIntakeOut()) {
            CommandScheduler.getInstance().schedule(new IntakeOutUpdateSetpoint(intake, pid, deltaTime));
        }
        //if setpoint won't kill the robot, we just set the setpoint
        else{
            pid.setSetpoint(setpoint);
        }
        //sets the arm power based on pid calculations
        arm.setMotorPower(pid.calculate(arm.getEncoderValue()));
        //if arm is out of the way, retract intake so we don't bang it into things
        if(arm.getEncoderValue() > Constants.ArmConstants.LOWER_BOUND_INTAKE_IN_TICKS){
            intake.retractIntake();
        }
    }
}
