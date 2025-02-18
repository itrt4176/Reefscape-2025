// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.EnumMap;

import frc.robot.subsystems.Claw;

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

  public static class ClawConstants {
    public static final int CLAW_ENCODER_CHANNEL = 2;

    public static final double STOW = 180.0;
    public static final double INTAKE = 200.0;
    public static final double LEVEL_ONE = 160.0;
    public static final double LEVEL_TWO = 220.0;
    public static final double LEVEL_THREE = 0.0;
    public static final double LEVEL_FOUR = 0.0;
  }
}
