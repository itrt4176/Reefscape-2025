// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.nio.file.attribute.AclEntryPermission;
import java.util.EnumMap;
import java.util.Map;

import frc.robot.subsystems.ArmJoint;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Pounds;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import swervelib.math.Matter;


/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // 32lbs * kg per pound
  public static final Matter CHASSIS    = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
  public static final double LOOP_TIME  = 0.13; //s, 20ms + 110ms sprk max velocity lag
  public static final double MAX_SPEED  = Units.feetToMeters(14.5);

  public static class OperatorConstants {
    public static final int DRIVE_CONTROLLER_PORT = 0;

    public static final double DEADBAND        = 0.1;
    public static final double LEFT_Y_DEADBAND = 0.1;
    public static final double RIGHT_X_DEADBAND = 0.1;
    public static final double LEFT_DEADBAND_X  = 0.1;
    public static final double LEFT_DEADBAND_Y  = 0.1;
    public static final double RIGHT_DEADBAND_X = 0.1;
    public static final double TURN_CONSTANT    = 6;

    public static final int ARM_CONTROL_PANEL_PORT = 1;
  }
  


  //ALL OF THESE ARE PLACEHOLDERS BEFORE WE CAN TEST THE BASE
  public static class DriveConstants {
    // public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // 32lbs * kg per pound
    public static final double ROBOT_MASS = Pounds.of(148 - 20.3).in(Kilograms);
    public static final Matter CHASSIS    = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
    public static final double LOOP_TIME  = 0.13; //s, 20ms + 110ms sprk max velocity lag
    public static final double MAX_SPEED  = Units.feetToMeters(14.5);
  }

  public static class ShoulderJointConstants {
    public static final int motorPort = 17;
    public static final int encoderPort = 0;
    public static final double encoderOffset = 81.0;

    public static final ArmJoint.PIDConfig pidConfig = new ArmJoint.PIDConfig(
      42.0,
      0.0,
      8.165, 
      160.0, 
      220.0, 
      0.5, 
      0.60459, 
      0.016192, 
      13.975,
      5.1899, 
      0.02
    );

    public static final Map<ArmJoint.Position, Double> angleMap = new EnumMap<>(ArmJoint.Position.class);

    static {
      angleMap.put(ArmJoint.Position.FLAT, 180.0);
      angleMap.put(ArmJoint.Position.INTAKE, 82.9);
      angleMap.put(ArmJoint.Position.LEVEL_ONE, 180.0);
      angleMap.put(ArmJoint.Position.LEVEL_TWO, 184.3);
      angleMap.put(ArmJoint.Position.LEVEL_THREE, 170.0);
      angleMap.put(ArmJoint.Position.LEVEL_FOUR, 104.0);
      angleMap.put(ArmJoint.Position.LOW_ALGAE, 140.0);
      angleMap.put(ArmJoint.Position.HIGH_ALGAE, 140.0);
      angleMap.put(ArmJoint.Position.START, 142.3);
    }
  }

  public static class ElbowJointConstants {
    public static final int motorPort = 41;
    public static final int encoderPort = 1;
    public static final double encoderOffset = 251.4;

    public static final ArmJoint.PIDConfig pidConfig = new ArmJoint.PIDConfig(
      54.392,
      0.0,
      1.45087, 
      160.0, 
      220.0, 
      0.5, 
      0.18808, 
      .025588, 
      23.908,
      2.5362, 
      0.02
    );
    public static final Map<ArmJoint.Position, Double> angleMap = new EnumMap<>(ArmJoint.Position.class);

    static {
      angleMap.put(ArmJoint.Position.FLAT, 0.0);
      angleMap.put(ArmJoint.Position.INTAKE, 202.8);
      angleMap.put(ArmJoint.Position.LEVEL_ONE, 0.0);
      angleMap.put(ArmJoint.Position.LEVEL_TWO, 28.3);
      angleMap.put(ArmJoint.Position.LEVEL_THREE, 54.3);
      angleMap.put(ArmJoint.Position.LEVEL_FOUR, 88.0);
      angleMap.put(ArmJoint.Position.LOW_ALGAE, -20.0);
      angleMap.put(ArmJoint.Position.HIGH_ALGAE, -20.0);
      angleMap.put(ArmJoint.Position.START, -51.2);
    }
  }


  public static class ClawConstants {
    public static final double CLAW_DEGREE_ROT_CONVERSION = 2.0;
    public static final double ENCODER_OFFSET = 206.0;

    public static final double INTAKE_ARC = 50.0;
    public static final double INTAKE_ROT = 90.0;

    public static final double L1_ARC = 65.0;
    public static final double L1_ROT = 90.0;

    public static final double L2_ARC = -17.8;
    public static final double L2_ROT = 0.0;

    public static final double L3_ARC = 30.0;
    public static final double L3_ROT = 0.0;

    public static final double L4_ARC = 45.0;
    public static final double L4_ROT = 0.0;

    public static final double LOW_ALGAE_ARC = 0.0;
    public static final double LOW_ALGAE_ROT = 0.0;

    public static final double HIGH_ALGAE_ARC = 0.0;
    public static final double HIGH_ALGAE_ROT = 0.0;

    public static final double FLAT_ARC = 65.0;
    public static final double FLAT_ROT = 0.0;
  }
}
