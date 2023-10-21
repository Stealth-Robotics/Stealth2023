package frc.robot.subsystems;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdleConfiguration;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class CandleSubsystem extends SubsystemBase {

    private final int LedCount = 308;

    private final CANdle candle = new CANdle(RobotMap.Candle.CANDLE);

    private int toRed = 0;
    private int toGreen = 255;
    private int toBlue = 0;

    Supplier<Gamepiece> gamepieceSupplier;
    BooleanSupplier beamBreak;

    public CandleSubsystem(Supplier<Gamepiece> gamepieceSupplier, BooleanSupplier beamBreak) {
        this.gamepieceSupplier = gamepieceSupplier;
        this.beamBreak = beamBreak;

        CANdleConfiguration config = new CANdleConfiguration();
        config.brightnessScalar = 1.0;
        config.disableWhenLOS = true;
        config.v5Enabled = false;
        config.enableOptimizations = true;
        config.statusLedOffWhenActive = false;

        candle.configAllSettings(config);

        candle.setLEDs(toRed, toGreen, toBlue, 0, 0, LedCount);

    }

    public void coneSolid() {

        toRed = 255;
        toGreen = 255;
        toBlue = 0;
    }

    public void gotPiece(){
        toRed = 0;
        toGreen = 255;
        toBlue = 0;
    }

    public void cubeSolid() {

        toRed = 240;
        toGreen = 10;
        toBlue = 180;
    }

    @Override
    public void periodic() {
        if(!beamBreak.getAsBoolean()){
            gotPiece();
        }
        else if(gamepieceSupplier.get() == Gamepiece.CONE)
        {
            coneSolid();
        }
        else {
            cubeSolid();
        }

        

        candle.setLEDs(toRed, toGreen, toBlue, 0, 0, LedCount);
        

    }
}