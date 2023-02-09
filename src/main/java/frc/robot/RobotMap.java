package frc.robot;

import edu.wpi.first.wpilibj.PneumaticsModuleType;

public class RobotMap {
    public static final class Drivebase {
        public static final int pigeonID = 2;

        /* Front Left Module - Module 0 */
        public static final class Mod0 {
            public static final int driveMotorID = 14;
            public static final int angleMotorID = 24;
            public static final int canCoderID = 34;
        }
        /* Front Right Module - Module 1 */
        public static final class Mod1 {
            public static final int driveMotorID = 12;
            public static final int angleMotorID = 22;
            public static final int canCoderID = 32;
        }

        /* Back Left Module - Module 2 */
        public static final class Mod2 {
            public static final int driveMotorID = 13;
            public static final int angleMotorID = 23;
            public static final int canCoderID = 33;
        }
        /* Back Right Module - Module 3 */
        public static final class Mod3 {
            public static final int driveMotorID = 11;
            public static final int angleMotorID = 21;
            public static final int canCoderID = 31;
        }
    }

    public static final class Pneumatics {
        public static final int PCM = 41;
        public static final PneumaticsModuleType PCM_TYPE = PneumaticsModuleType.CTREPCM;
        public static final int INTAKE_DEPLOY_PCM_CHANNEL = 0;
    }

}