package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotMap;

import com.ctre.phoenix.led.*;

public class CANdleSubsystem extends SubsystemBase {
    private final CANdle candle = new CANdle(RobotMap.CANdle.CANDLE);

    private Animation toAnimate;

    private int redLevel;
    private int blueLevel;
    private int greenLevel;
    private int whiteLevel;

    public CANdleSubsystem() {
        toAnimate = new RainbowAnimation(1, 0.5, Constants.CANdleConstants.LED_COUNT);

        CANdleConfiguration config = new CANdleConfiguration();
        config.brightnessScalar = 1.0;
        candle.configAllSettings(config);
    }

    public void yellow() {
        toAnimate = null;
        redLevel = 255;
        greenLevel = 255;
        blueLevel = 0;
        whiteLevel = 0;
    }

    public void purple() {
        toAnimate = null;
        redLevel = 255;
        greenLevel = 0;
        blueLevel = 255;
        whiteLevel = 0;
    }

    public void idle() {
        toAnimate = new RainbowAnimation(1.0, 0.5, Constants.CANdleConstants.LED_COUNT);
    }

    @Override
    public void periodic() {
        if (toAnimate != null) {
            candle.animate(toAnimate);
        } else {
            candle.setLEDs(redLevel, blueLevel, greenLevel, whiteLevel, 0, Constants.CANdleConstants.LED_COUNT);
        }
    }
}