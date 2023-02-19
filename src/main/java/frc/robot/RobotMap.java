package frc.robot;

import edu.wpi.first.wpilibj.PneumaticsModuleType;

public class RobotMap {
    public static final class Pneumatics {
        //TODO: Change to fit the IDs on the robot.
        public static final int PCM = 41;
        public static final PneumaticsModuleType PCM_TYPE = PneumaticsModuleType.CTREPCM;
        public static final int INTAKE_DEPLOY_PCM_CHANNEL = 0;
    }
    public static final class IntakeIDs {
        //TODO: Change to fit the IDs on the robot.
        public static final int RIGHT_INTAKE_MOTOR_ID = 0;
        public static final int LEFT_INTAKE_MOTOR_ID = 1;
    }

}