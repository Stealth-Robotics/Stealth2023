package frc.robot.subsystems;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.SingleFadeAnimation;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class CandleSubsystem extends SubsystemBase {

    private final int LedCount = 87;

    private final CANdle candle = new CANdle(RobotMap.Candle.CANDLE);

    private int toRed = 0;
    private int toGreen = 0;
    private int toBlue = 255;

    public CandleSubsystem() {
        CANdleConfiguration config = new CANdleConfiguration();
        config.brightnessScalar = 1.0;
        config.disableWhenLOS = true;
        config.v5Enabled = false;
        config.enableOptimizations = true;
        config.statusLedOffWhenActive = false;

        candle.configAllSettings(config);
    }

    public void cone() {

        toRed = 255;
        toGreen = 255;
        toBlue = 0;
    }

    public void cube() {

        toRed = 240;
        toGreen = 10;
        toBlue = 180;
    }

    @Override
    public void periodic() {

        candle.setLEDs(toRed, toGreen, toBlue, 0, 0, LedCount);
    }
}