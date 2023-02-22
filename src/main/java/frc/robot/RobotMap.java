package frc.robot;

import edu.wpi.first.wpilibj.PneumaticsModuleType;

public class RobotMap {

    public static final class Claw {
        public static final int TIME_OF_FLIGHT = 0;
    }
    public static final class Drivebase {
        public static final int PIGEON_ID = 2;

        /* Front Left Module - Module 0 */
        public static final class Mod0 {
            public static final int DRIVE_MOTOR_ID = 14;
            public static final int ANGLE_MOTOR_ID = 24;
            public static final int CANCODER_ID = 34;
        }
        /* Front Right Module - Module 1 */
        public static final class Mod1 {
            public static final int DRIVE_MOTOR_ID = 12;
            public static final int ANGLE_MOTOR_ID = 22;
            public static final int CANCODER_ID = 32;
        }

        /* Back Left Module - Module 2 */
        public static final class Mod2 {
            public static final int DRIVE_MOTOR_ID = 13;
            public static final int ANGLE_MOTOR_ID = 23;
            public static final int CANCODER_ID = 33;
        }
        /* Back Right Module - Module 3 */
        public static final class Mod3 {
            public static final int DRIVE_MOTOR_ID = 11;
            public static final int ANGLE_MOTOR_ID = 21;
            public static final int CANCODER_ID = 31;
        }
    }

    public static final class Pneumatics {
        public static final int PCM = 41;
        public static final PneumaticsModuleType PCM_TYPE = PneumaticsModuleType.CTREPCM;
        public static final int INTAKE_DEPLOY_PCM_CHANNEL = 0;
        public static final int CLAW_PCM_CHANNEL = 0;
    }

}