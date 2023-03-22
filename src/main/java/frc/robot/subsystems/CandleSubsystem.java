package frc.robot.subsystems;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.SingleFadeAnimation;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class CandleSubsystem extends SubsystemBase {

    private final int LedCount = 308;
    private final SingleFadeAnimation coneBlinkAnimation = new SingleFadeAnimation(255, 255, 0, 0, 0.4, LedCount);
    private final SingleFadeAnimation cubeBlinkAnimation = new SingleFadeAnimation(240, 10, 180, 0, 0.4, LedCount);

    private final CANdle candle = new CANdle(RobotMap.Candle.CANDLE);

    private Animation toAnimate = new RainbowAnimation(1, 0.5, LedCount);

    private int toRed = 0;
    private int toGreen = 0;
    private int toBlue = 0;

    public CandleSubsystem() {
        CANdleConfiguration config = new CANdleConfiguration();
        config.brightnessScalar = 1.0;
        config.disableWhenLOS = true;
        config.v5Enabled = false;
        config.enableOptimizations = true;
        config.statusLedOffWhenActive = false;

        candle.configAllSettings(config);
    }

    public void coneBlink() {
        toAnimate = coneBlinkAnimation;
    }

    public void cubeBlink() {
        toAnimate = cubeBlinkAnimation;
    }

    public void coneSolid() {
        toAnimate = null;

        toRed = 255;
        toGreen = 255;
        toBlue = 0;
    }

    public void cubeSolid() {
        toAnimate = null;

        toRed = 240;
        toGreen = 10;
        toBlue = 180;
    }

    @Override
    public void periodic() {

        if (toAnimate == null) {
            candle.setLEDs(toRed, toGreen, toBlue, 0, 0, LedCount);
        }

        else {
            candle.animate(toAnimate);
        }

    }
}
