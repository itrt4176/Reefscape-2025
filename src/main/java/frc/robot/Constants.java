// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.EnumMap;
import java.util.Map;

import frc.robot.subsystems.arm.ArmJointPIDConfig;
import frc.robot.subsystems.arm.ArmPosition;

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

  public static class ArmJoint1Constants {
    public static final int motorPort = 41;
    public static final int encoderPort = 0;

    public static final ArmJointPIDConfig pidConfig = new ArmJointPIDConfig(
      0.016,
      0.0,
      0.001, 
      80.0, 
      160.0, 
      0.5, 
      1.1, 
      1.2, 
      80.0, 
      0.02
    );

    public static final Map<ArmPosition, Double> angleMap = new EnumMap<>(ArmPosition.class);

    static {
      // PLACEHOLDERS
      angleMap.put(ArmPosition.STOW, 0.0);
      angleMap.put(ArmPosition.INTAKE, 162.0);
      angleMap.put(ArmPosition.LEVEL_ONE, 200.0);
      angleMap.put(ArmPosition.LEVEL_TWO, 190.0);
      angleMap.put(ArmPosition.LEVEL_THREE, 180.0);
      angleMap.put(ArmPosition.LEVEL_FOUR, 170.0);
    }
  }

  public static class ArmJoint2Constants {
    public static final int motorPort = 17;
    public static final int encoderPort = 2;

    public static final ArmJointPIDConfig pidConfig = new ArmJointPIDConfig(
      0.016,
      0.0,
      0.0, 
      80.0, 
      160.0, 
      0.5, 
      1.1, 
      1.2, 
      80.0, 
      0.02
    );
    public static final Map<ArmPosition, Double> angleMap = new EnumMap<>(ArmPosition.class);

    static {
      // PLACEHOLDERS
      angleMap.put(ArmPosition.STOW, 0.0);
      angleMap.put(ArmPosition.INTAKE, 306.0);
      angleMap.put(ArmPosition.LEVEL_ONE, 160.0);
      angleMap.put(ArmPosition.LEVEL_TWO, 170.0);
      angleMap.put(ArmPosition.LEVEL_THREE, 180.0);
      angleMap.put(ArmPosition.LEVEL_FOUR, 190.0);
    }
  }
}
