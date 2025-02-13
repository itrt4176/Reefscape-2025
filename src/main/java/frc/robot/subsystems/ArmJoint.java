// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;

import java.util.Map;
import java.util.function.DoubleSupplier;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmJoint extends SubsystemBase {
  public enum Position {
    STOW,
    INTAKE,
    LEVEL_ONE,
    LEVEL_TWO,
    LEVEL_THREE,
    LEVEL_FOUR
  }

  public record PIDConfig(
    double p,
    double i,
    double d,
    double maxVelocity,
    double maxAcceleration,
    double tolerance,
    double s,
    double g,
    double v,
    double loopTime
  ) {}

  private AnalogEncoder jointEncoder;
  private SparkMax jointMotor;

  private MutAngle angle;
  private MutAngle goal;
  private MutAngle offset;

  private ProfiledPIDController pid;
  private ArmFeedforward ff;

  private final Map<Position, Double> angleMap;

  private boolean enabled;

/** Creates a new ArmSubsystem. */
  public ArmJoint(int motorId, int encoderId, PIDConfig pidConfig, Map<Position, Double> angleMap, String name) {
    super(name);
    jointEncoder = new AnalogEncoder(encoderId);
    jointMotor = new SparkMax(motorId, MotorType.kBrushless);

    this.angleMap = angleMap;

    angle = Degrees.mutable(jointEncoder.get() * 360.0);
    goal = Degrees.mutable(angleMap.get(Position.STOW));
    offset = Degrees.mutable(0.0);

    pid = new ProfiledPIDController(
      pidConfig.p(),
      pidConfig.i(),
      pidConfig.d(),
      new TrapezoidProfile.Constraints(
        pidConfig.maxVelocity(),
        pidConfig.maxAcceleration()
      ),
      pidConfig.loopTime()
    );

    pid.setTolerance(pidConfig.tolerance());

    ff = new ArmFeedforward(
      pidConfig.s(),
      pidConfig.g(), 
      pidConfig.v(),
      pidConfig.loopTime()
    );

    enabled = true;
  }

  public Angle getAngle() {
    return angle;
  }

  private void updateAngle() {
    angle.mut_replace(jointEncoder.get() * 360.0, Degrees);
  }

  public Angle getGoal() {
    return goal;
  }

  public Angle getOffset() {
    return offset;
  }

  public Command setPosition(Position position) {
    return runOnce(() -> {
      goal.mut_setMagnitude(angleMap.get(position));
      pid.setGoal(goal.in(Radians) + offset.in(Radians));
    }).withName(position.toString());
  }

  public Command adjustOffset(DoubleSupplier offsetSupplier) {
    return runOnce(() -> {
      offset.mut_plus(offsetSupplier.getAsDouble() * 0.1, Degrees);
      pid.setGoal(goal.in(Radians) + offset.in(Radians));
    }).withName("Adjust Offset");
  }

  public boolean isEnabled() {
    return enabled;
  }

  public void setEnabled(boolean enabled) {
    this.enabled = enabled;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    updateAngle();

    if (enabled && !pid.atGoal()) {
      jointMotor.setVoltage(
        pid.calculate(angle.in(Radians))
          + ff.calculate(pid.getSetpoint().position, pid.getSetpoint().velocity)
      );
    }

    SmartDashboard.putNumber(this.getName() + " Position", angle.in(Degrees));
  }
}
