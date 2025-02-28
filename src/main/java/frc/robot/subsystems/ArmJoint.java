// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import java.util.Map;
import java.util.function.DoubleSupplier;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularAcceleration;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.utils.AccumulatingAnalogEncoder;
import frc.robot.utils.BrakingMotors;

@Logged
public class ArmJoint extends SubsystemBase implements BrakingMotors {
  public enum Position {
    CLIMB,
    INTAKE,
    LEVEL_ONE,
    LEVEL_TWO,
    LEVEL_THREE,
    LEVEL_FOUR,
    LOW_ALGAE,
    HIGH_ALGAE,
    START
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
    double a,
    double loopTime
  ) {}

  private SparkMax jointMotor;
  private AccumulatingAnalogEncoder jointEncoder;
  private double encoderOffset;

  private MutAngle angle;
  private DoubleSupplier relativeToAngle;
  private MutAngularVelocity velocity;
  private MedianFilter velocityFilter;
  private MutAngularAcceleration acceleration;
  private MedianFilter accelFilter;
  private double timestamp;
  
  private MutAngle goal;
  private MutAngle goalAdjustment;

  private ProfiledPIDController pid;
  private ArmFeedforward ff;

  private final Map<Position, Double> angleMap;

  private final Trigger atGoal;

  private final SysIdRoutine idRoutine;
  private final MutVoltage routineVoltage = Volts.mutable(0);

  private SparkBaseConfig jointConfig;

  private boolean enabled;
  private boolean sysIdRunning;

  private boolean firstRun;

/** Creates a new ArmSubsystem. */
  public ArmJoint(int motorId, int encoderId, double encoderOffset,  DoubleSupplier relativeToAngle,  PIDConfig pidConfig, Map<Position, Double> angleMap, String name, boolean isInverted, double minAngle, double maxAngle) {
    super(name);

    firstRun = true;

    jointMotor = new SparkMax(motorId, MotorType.kBrushless);
    jointEncoder = new AccumulatingAnalogEncoder(encoderId, 5);
    this.encoderOffset = encoderOffset;
    this.relativeToAngle = relativeToAngle;

    jointConfig = new SparkMaxConfig()
      .inverted(isInverted)
      .idleMode(IdleMode.kCoast);

    jointMotor.configure(jointConfig, null, PersistMode.kPersistParameters);

    this.angleMap = angleMap;

    angle = Degrees.mutable(jointEncoder.get() * 360.0 - encoderOffset);
    velocity = DegreesPerSecond.mutable(0);
    velocityFilter = new MedianFilter(15);
    acceleration = DegreesPerSecondPerSecond.mutable(0);
    accelFilter = new MedianFilter(15);
    timestamp = Timer.getFPGATimestamp();

    goal = Degrees.mutable(angleMap.get(Position.START));
    goalAdjustment = Degrees.mutable(0.0);

    pid = new ProfiledPIDController(
      pidConfig.p(),
      pidConfig.i(),
      pidConfig.d(),
      new TrapezoidProfile.Constraints(
        pidConfig.maxVelocity() / 360.0,
        pidConfig.maxAcceleration() / 360.0
      ),
      pidConfig.loopTime()
    );

    pid.setTolerance(pidConfig.tolerance() / 360.0);

    ff = new ArmFeedforward(
      pidConfig.s(),
      pidConfig.g(), 
      pidConfig.v(),
      pidConfig.a(),
      pidConfig.loopTime()
    );

    pid.setGoal(goal.in(Rotations));

    
    atGoal = new Trigger(pid::atGoal);

    setDefaultCommand(run(this::moveJoint));

    idRoutine = new SysIdRoutine(
      new SysIdRoutine.Config(
        Volts.of(1).per(Second),
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

  public ArmJoint(int motorId, int encoderId, double encoderOffset, PIDConfig pidConfig, Map<Position, Double> angleMap, String name, boolean isInverted, double minAngle, double maxAngle) {
    this(motorId, encoderId, encoderOffset, () -> 0, pidConfig, angleMap, name, isInverted, minAngle, maxAngle);
  }

  public Angle getAngle() {
    return angle;
  }

  private void updateAngle() {
    double newTimestamp = Timer.getFPGATimestamp();
    double period = newTimestamp - timestamp;
    double newAngle = jointEncoder.get() * 360.0 - encoderOffset + relativeToAngle.getAsDouble();
    double newVelocity = velocityFilter.calculate((newAngle - angle.in(Degrees)) / period);
    double newAcceleration = accelFilter.calculate((newVelocity - velocity.in(DegreesPerSecond)) / period);

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

  public Angle getGoalAdjustment() {
    return goalAdjustment;
  }

  public Command setPosition(Position position) {
    return runOnce(() -> {
      goal.mut_replace(angleMap.get(position), Degrees);
      goalAdjustment.mut_replace(0, Degrees);
      pid.setGoal(goal.in(Rotations) + goalAdjustment.in(Rotations));
      pid.reset(angle.in(Rotations), velocity.in(RotationsPerSecond));
    }).asProxy().withName(position.toString());
  }

  public Command adjustGoal(DoubleSupplier offsetSupplier) {
    return runOnce(() -> pid.reset(angle.in(Rotations), velocity.in(RotationsPerSecond))).andThen(
      runOnce(() -> {
        var adjustment = goalAdjustment.in(Degrees);
        var offsetInput = MathUtil.applyDeadband(offsetSupplier.getAsDouble(), 0.1);
        var newAdjustment = adjustment + (offsetInput * 2.0 / 50.0);
        goalAdjustment.mut_replace(newAdjustment, Degrees);
        pid.setGoal(goal.in(Rotations) + goalAdjustment.in(Rotations));
        // Maybe this is the use case for setting a new goal without resetting the PID loop?
        // pid.reset(angle.in(Rotations), velocity.in(RotationsPerSecond));
      }).andThen(this::moveJoint, this).repeatedly()
    ).withName(getName() + " Adjust Goal");
  }

  public void setOffset(DoubleSupplier offsetSupplier) {
    goalAdjustment.mut_plus(offsetSupplier.getAsDouble(), Degrees);
    pid.setGoal(goal.in(Rotations) + goalAdjustment.in(Rotations));
    pid.reset(angle.in(Rotations), velocity.in(RotationsPerSecond));
  }

  private void moveJoint() {
    // Move joint only runs for the first time after the robot is enabled
    // for the first time. The ProfiledPIDController considers time since
    // the last reset. This caused the "fist bump". To fix, reset the PID
    // the first time moveJoint() is called.
    if (firstRun) {
      pid.reset(angle.in(Rotations), velocity.in(RotationsPerSecond));
      firstRun = false;
    }

    var setpoint = pid.getSetpoint();

    double motorVoltage = pid.calculate(angle.in(Rotations))
        + ff.calculate(setpoint.position, setpoint.velocity);

        SmartDashboard.putNumber(getName() + " Goal", goal.in(Degrees));
        SmartDashboard.putNumber(getName() + " Position Setpoint", setpoint.position * 360.0);
        SmartDashboard.putNumber(getName() + " Velocity Setpoint", setpoint.velocity * 360.0);

    if (!sysIdRunning) {
      if (enabled && !pid.atGoal()) {
        jointMotor.setVoltage(motorVoltage);
      } else {
        jointMotor.set(0.0);
      }
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
  public Command enableMotorBrakes(boolean enable) {
    var idleMode = enable ? IdleMode.kBrake : IdleMode.kCoast;

    return runOnce(() -> jointMotor.configure(jointConfig.idleMode(idleMode), ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters))
      .ignoringDisable(true);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (!sysIdRunning) {
      updateAngle();
    }

    SmartDashboard.putNumber(getName() + " Raw Encoder", jointEncoder.get());
    SmartDashboard.putNumber(getName() + " Unwrapped Encoder", jointEncoder.getRaw());
    SmartDashboard.putNumber(getName() + " Position", angle.in(Degrees));
    SmartDashboard.putNumber(getName() + " Velocity", velocity.in(DegreesPerSecond));
    SmartDashboard.putNumber(getName() + " Acceleration", acceleration.in(DegreesPerSecondPerSecond));
    SmartDashboard.putNumber(getName() + " Motor Bus Voltage", jointMotor.getBusVoltage());
    SmartDashboard.putNumber(getName() + " Motor get()", jointMotor.get());
    SmartDashboard.putNumber(getName() + " Motor Voltage", jointMotor.get() * jointMotor.getBusVoltage());
    SmartDashboard.putNumber(getName() + " Goal", goal.in(Degrees));
    SmartDashboard.putNumber(getName() + " Manual Adjustment", goalAdjustment.in(Degrees));
  }
}
