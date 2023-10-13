package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class IntakeSubsystem extends SubsystemBase{
    private final WPI_TalonFX intake;
    private final DigitalInput beamBreak;
    private Gamepiece gamePiece = Gamepiece.CONE;

    public IntakeSubsystem(){
        intake = new WPI_TalonFX(RobotMap.Crocodile.INTAKE);
        beamBreak = new DigitalInput(RobotMap.Crocodile.BEAM_BREAK_ID);
        intake.setInverted(true);
    }

    public void setIntakeSpeed(double speed) {
        intake.set(speed);
    }

    public boolean getBeamBreak() {
        return beamBreak.get();
    }
    
    public Gamepiece getGamePiece() {
        return gamePiece;
    }

    public void setGamePiece(Gamepiece gamePiece) {
        this.gamePiece = gamePiece;
    }
    
}
