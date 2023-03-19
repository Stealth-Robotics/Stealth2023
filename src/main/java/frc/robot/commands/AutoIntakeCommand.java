package frc.robot.commands;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.CrocodileSubsystem;

public class AutoIntakeCommand extends CommandBase {
    private final CrocodileSubsystem crocodileSubsystem;
    private final double speed;
    private final Timer timer;
    private final Debouncer debouncer;

    public AutoIntakeCommand(CrocodileSubsystem crocodileSubsystem, double speed/*, boolean intakeLonger*/) {
        this.crocodileSubsystem = crocodileSubsystem;
        this.speed = speed;
        debouncer = new Debouncer(0.5, DebounceType.kBoth);
        timer = new Timer();
        addRequirements(crocodileSubsystem);
    }

    @Override
    public void initialize() {
        crocodileSubsystem.setIntakeSpeed(speed);
    }

    @Override
    public boolean isFinished() {
        //if outtaking, keep running motors until beam break hasn't been broken for 0.5 seconds
        if(speed < 0){
            if(crocodileSubsystem.getBeamBreak()){
                timer.start();
            }
            if(timer.hasElapsed(0.5)){
                timer.stop();
                timer.reset();
                return true;
            }
        }
        //otherwise, keep running motors until beam break has been broken 
        else if(!crocodileSubsystem.getBeamBreak()){
            return  true;
        }
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        crocodileSubsystem.setIntakeSpeed(0);
    }

}
