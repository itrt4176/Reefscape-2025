// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import static edu.wpi.first.units.Units.Degrees;

import java.util.EnumMap;
import java.util.Map;
import java.util.function.DoubleSupplier;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmJoint1 extends SubsystemBase {
  private AnalogEncoder jointEncoder;
  private SparkMax jointMotor;

  private MutAngle angle;
  private MutAngle setpoint;
  private MutAngle offset;

  private static final Map<ArmPosition, Double> angleMap;

  static {
    angleMap = new EnumMap<>(ArmPosition.class);
    angleMap.put(ArmPosition.STOW, 0.0);
    angleMap.put(ArmPosition.INTAKE, 162.0);
    angleMap.put(ArmPosition.LEVEL_ONE, 200.0);
    angleMap.put(ArmPosition.LEVEL_TWO, 190.0);
    angleMap.put(ArmPosition.LEVEL_THREE, 180.0);
    angleMap.put(ArmPosition.LEVEL_FOUR, 170.0);
  }

  /** Creates a new ArmSubsystem. */
  public ArmJoint1() {
    jointEncoder = new AnalogEncoder(0);
    jointMotor = new SparkMax(41, MotorType.kBrushless);

    angle = Degrees.mutable(jointEncoder.get() * 360.0);
    setpoint = Degrees.mutable(angleMap.get(ArmPosition.STOW));
    offset = Degrees.mutable(0.0);
  }

  public Angle getAngle() {
    return angle.mut_setMagnitude(jointEncoder.get() * 360.0);
  }

  public Angle getSetpoint() {
    return setpoint;
  }

  public Angle getOffset() {
    return offset;
  }

  public Command setPosition(ArmPosition position) {
    return runOnce(() -> {
      setpoint.mut_setMagnitude(angleMap.get(position));
    }).withName(position.toString());
  }

  public Command adjustOffset(DoubleSupplier offsetSupplier) {
    return runOnce(() -> {
      offset.mut_plus(offsetSupplier.getAsDouble() * 0.1, Degrees);
    }).withName("Adjust Offset");
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    SmartDashboard.putNumber("Arm Joint 1 Position", angle.in(Degrees));
  }
}
