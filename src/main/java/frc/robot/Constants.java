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
  public static final int xboxPort = 0;
  public static final double DEAD_ZONE = 0.05;

  public static final int FRONT_RIGHT_CANCODER = 1;
  public static final int FRONT_LEFT_CANCODER = 2;
  public static final int BACK_RIGHT_CANCODER = 3;
  public static final int BACK_LEFT_CANCODER = 4;

  public static final int FRONT_RIGHT_DRIVE = 11;
  public static final int FRONT_RIGHT_DIRECTION = 12; 
  public static final int FRONT_LEFT_DRIVE = 21;
  public static final int FRONT_LEFT_DIRECTION = 22;
  public static final int BACK_RIGHT_DRIVE = 31;
  public static final int BACK_RIGHT_DIRECTION = 32;
  public static final int BACK_LEFT_DRIVE = 41;
  public static final int BACK_LEFT_DIRECTION = 42;

  public static final double[] FRONT_RIGHT_R = {1,1};
  public static final double[] FRONT_LEFT_R = {-1,1};
  public static final double[] BACK_RIGHT_R = {1,-1};
  public static final double[] BACK_LEFT_R = {-1,-1};

  public static final double ENCODER_COUNTS_PER_REV = 150/7;
}
