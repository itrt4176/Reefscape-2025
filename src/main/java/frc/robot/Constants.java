// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.EnumMap;
import java.util.Map;

import frc.robot.subsystems.ArmJoint;

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

  public static class ShoulderJointConstants {
    public static final int motorPort = 17;
    public static final int encoderPort = 0;
    public static final double encoderOffset = 81.0;

    public static final ArmJoint.PIDConfig pidConfig = new ArmJoint.PIDConfig(
      42.0,
      0.0,
      7.265, 
      160.0, 
      320.0, 
      0.5, 
      0.60459, 
      0.016192, 
      13.975,
      5.1899, 
      0.02
    );

    public static final Map<ArmJoint.Position, Double> angleMap = new EnumMap<>(ArmJoint.Position.class);

    static {
      // PLACEHOLDERS
      angleMap.put(ArmJoint.Position.STOW, 185.2);
      angleMap.put(ArmJoint.Position.INTAKE, 81.0);//maybe
      angleMap.put(ArmJoint.Position.LEVEL_ONE, 119.0);//placeholder
      angleMap.put(ArmJoint.Position.LEVEL_TWO, 137.5);//placeholder for testing
      angleMap.put(ArmJoint.Position.LEVEL_THREE, 99.0);//placeholder
      angleMap.put(ArmJoint.Position.LEVEL_FOUR, 89.85);
    }
  }

  public static class ElbowJointConstants {
    public static final int motorPort = 41;
    public static final int encoderPort = 1;
    public static final double encoderOffset = 188.6;

    public static final ArmJoint.PIDConfig pidConfig = new ArmJoint.PIDConfig(
      54.392,
      0.0,
      0.45087, 
      160.0, 
      320.0, 
      0.5, 
      0.18808, 
      .025588, 
      23.908,
      2.5362, 
      0.02
    );
    public static final Map<ArmJoint.Position, Double> angleMap = new EnumMap<>(ArmJoint.Position.class);

    static {
      // PLACEHOLDERS
      angleMap.put(ArmJoint.Position.STOW, 0.0);
      angleMap.put(ArmJoint.Position.INTAKE, 306.0);
      angleMap.put(ArmJoint.Position.LEVEL_ONE, 160.0);
      angleMap.put(ArmJoint.Position.LEVEL_TWO, 90.0);
      angleMap.put(ArmJoint.Position.LEVEL_THREE, 180.0);
      angleMap.put(ArmJoint.Position.LEVEL_FOUR, 180.0);
    }
  }
}
