// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }


  public final class RotatorConstants
  {
    // Radians Per Second
    public static final double MAX_VELOCITY = 3.0;
    // Radians Per Second Squared
    public static final double MAX_ACCELERATION = 10.0;

    // TODO: Tune After Feedforward is tuned
    public static final double ROTATOR_P_COEFF = 1.0;
    public static final double ROTATOR_I_COEFF = 0.0;
    public static final double ROTATOR_D_COEFF = 0.0;

    // TODO: Gain values through SysID
    public static final double ROTATOR_KS_COEFF = 1.0;
    public static final double ROTATOR_KG_COEFF = 1.0;
    // Volt Second Per Rad
    public static final double ROTATOR_KV_COEFF = 0.0;
    // Volt Second Squared Per Rad
    public static final double ROTATOR_KA_COEFF = 0.0;

    

    // TODO: Measure and set encoder offset to make 0 perfectly horizontal
    public static final double ENCODER_OFFSET = 0.0;
    // TODO: Verify
    public static final double ENCODER_CPR = 2048.0;
    // TODO: Measure and set gear ratio between the output shaft and the encoder
    public static final double GEAR_RATIO = 1.0 / 1.0;
}

  public final class ArmConstants {
    
    // Radians Per Second
    public static final double MAX_VELOCITY = 3.0;
    // Radians Per Second Squared
    public static final double MAX_ACCELERATION = 10.0;


    public static final double ARM_P_COEFF = 1.0;
    public static final double ARM_I_COEFF = 0.0;
    public static final double ARM_D_COEFF = 0.0;

    
    // TODO: Measure and set encoder offset to make 0 perfectly horizontal
    public static final double ENCODER_OFFSET = 0.0;
    // TODO: Verify
    public static final double ENCODER_CPR = 2048.0;
    // TODO: Measure and set gear ratio between the output shaft and the encoder
    public static final double GEAR_RATIO = 1.0 / 1.0;

  }
}
