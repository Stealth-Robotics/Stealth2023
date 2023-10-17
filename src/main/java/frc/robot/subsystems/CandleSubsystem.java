package frc.robot.subsystems;

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

    public CandleSubsystem() {
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

    public void cubeSolid() {

        toRed = 240;
        toGreen = 10;
        toBlue = 180;
    }

    @Override
    public void periodic() {
            candle.setLEDs(toRed, toGreen, toBlue, 0, 0, LedCount);
        

    }
}