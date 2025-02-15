// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import java.util.Map;
import java.util.function.DoubleSupplier;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularAcceleration;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

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
  private MutAngularVelocity velocity;
  private MutAngularAcceleration acceleration;
  private double timestamp;
  
  private MutAngle goal;
  private MutAngle offset;

  private ProfiledPIDController pid;
  private ArmFeedforward ff;

  private final Map<Position, Double> angleMap;

  private final Trigger atGoal;

  private final SysIdRoutine idRoutine;
  private final MutVoltage routineVoltage = Volts.mutable(0);

  private boolean enabled;
  private boolean sysIdRunning;

/** Creates a new ArmSubsystem. */
  public ArmJoint(int motorId, int encoderId, PIDConfig pidConfig, Map<Position, Double> angleMap, String name) {
    super(name);
    jointEncoder = new AnalogEncoder(encoderId);
    jointMotor = new SparkMax(motorId, MotorType.kBrushless);

    this.angleMap = angleMap;

    angle = Degrees.mutable(jointEncoder.get() * 360.0);
    velocity = DegreesPerSecond.mutable(0);
    acceleration = DegreesPerSecondPerSecond.mutable(0);
    timestamp = Timer.getFPGATimestamp();

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

    pid.setTolerance(pidConfig.tolerance() / 360.0);

    ff = new ArmFeedforward(
      pidConfig.s(),
      pidConfig.g(), 
      pidConfig.v(),
      pidConfig.loopTime()
    );

    atGoal = new Trigger(pid::atGoal);

    setDefaultCommand(run(this::moveJoint));

    idRoutine = new SysIdRoutine(
      new SysIdRoutine.Config(
        Volts.of(2).per(Second),
        Volts.of(8),
        null
      ),
      new SysIdRoutine.Mechanism(
        volts -> { jointMotor.set(volts.magnitude() / jointMotor.getBusVoltage()); },
        log -> {
          updateAngle();

          log.motor(getName() + " motor")
            .voltage(routineVoltage.mut_replace(jointMotor.get() * jointMotor.getBusVoltage(), Volts))
            .angularPosition(angle)
            .angularVelocity(velocity)
            .angularAcceleration(acceleration);
        },
        this
      )
    );

    enabled = true;
    sysIdRunning = false;
  }

  public Angle getAngle() {
    return angle;
  }

  private void updateAngle() {
    double newTimestamp = Timer.getFPGATimestamp();
    double period = newTimestamp - timestamp;
    double newAngle = jointEncoder.get() * 360.0;
    double newVelocity = (newAngle - angle.in(Degrees)) / period;
    double newAcceleration = (newVelocity - velocity.in(DegreesPerSecond)) / period;

    acceleration.mut_replace(newAcceleration, DegreesPerSecondPerSecond);
    velocity.mut_replace(newVelocity, DegreesPerSecond);
    angle.mut_replace(newAngle, Degrees);

    timestamp = newTimestamp;
  }

  public Angle getGoal() {
    return goal;
  }

  public Trigger atGoal() {
    return atGoal;
  }

  public Angle getOffset() {
    return offset;
  }

  public Command setPosition(Position position) {
    return runOnce(() -> {
      goal.mut_replace(angleMap.get(position), Degrees);
      pid.setGoal(goal.in(Rotations) + offset.in(Rotations));
    }).withName(position.toString());
  }

  public Command adjustOffset(DoubleSupplier offsetSupplier) {
    return runOnce(() -> {
      offset.mut_plus(offsetSupplier.getAsDouble() * 0.1, Degrees);
      pid.setGoal(goal.in(Rotations) + offset.in(Rotations));
    }).withName("Adjust Offset")
      .andThen(this::moveJoint, this)
      .repeatedly();
  }

  private void moveJoint() {
    if (enabled && !sysIdRunning && !pid.atGoal()) {
      jointMotor.setVoltage(
        pid.calculate(angle.in(Rotations))
          + ff.calculate(pid.getSetpoint().position, pid.getSetpoint().velocity)
      );
    }
  }

  private Command sysIdCommand(Command command) {
    return runOnce(() -> { sysIdRunning = true; })
      .andThen(command)
      .andThen(() -> { sysIdRunning = false; });
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return sysIdCommand(idRoutine.quasistatic(direction));
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return sysIdCommand(idRoutine.dynamic(direction));
  }

  public boolean isEnabled() {
    return enabled;
  }

  /**
   * Enable/disable the arm joint for tuning and debugging purposes
   * 
   * @param enabled Whether or not the arm joint is enabled
   */
  public void setEnabled(boolean enabled) {
    this.enabled = enabled;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (!sysIdRunning) {
      updateAngle();
    }

    SmartDashboard.putNumber(getName() + " Raw Encoder", jointEncoder.get());
    SmartDashboard.putNumber(getName() + " Position", angle.in(Degrees));
    SmartDashboard.putNumber(getName() + " Velocity", velocity.in(DegreesPerSecond));
    SmartDashboard.putNumber(getName() + " Acceleration", acceleration.in(DegreesPerSecondPerSecond));
    SmartDashboard.putNumber(getName() + " Motor Bus Voltage", jointMotor.getBusVoltage());
    SmartDashboard.putNumber(getName() + " Motor get()", jointMotor.get());
    SmartDashboard.putNumber(getName() + " Motor Voltage", jointMotor.get() * jointMotor.getBusVoltage());
  }
}
