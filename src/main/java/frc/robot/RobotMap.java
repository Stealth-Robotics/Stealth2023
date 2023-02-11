package frc.robot;


import edu.wpi.first.wpilibj.PneumaticsModuleType;

public class RobotMap {
    public static final class Pneumatics {
        public static final int PCM = 41;
        public static final PneumaticsModuleType PCM_TYPE = PneumaticsModuleType.CTREPCM;
        public static final int INTAKE_DEPLOY_PCM_CHANNEL = 0;
    }

    public static final class Arm {
        public static final int ROTATOR_MOTOR = 1;
        
        public static final int ENCODER_PORT = 0;
    }

}