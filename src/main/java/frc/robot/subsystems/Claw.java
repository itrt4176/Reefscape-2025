// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.Constants.ClawConstants;
import frc.robot.commands.HomeWrist;
import frc.robot.utils.BrakingMotors;

@Logged
public class Claw extends SubsystemBase implements BrakingMotors {
  
  /** 
   * The left motor controller controlling the wrist
   * */ 
  SparkMax leftMotor;

  /** 
   * The right motor controller controlling the wrist
   * */ 
  SparkMax rightMotor;

  private SparkMax gripMotor;

  AnalogEncoder arcThrift;

  SparkMaxConfig leftConfig = new SparkMaxConfig();
  SparkMaxConfig rightConfig = new SparkMaxConfig();

  DigitalInput rotationInput;

  DigitalInput detectStick;

  private final Trigger atHome = new Trigger(this::isRotationHomed);

  private final PIDController arcPID = new PIDController(0.014, 0.000, 0.0);
  private final Trigger atArcSetpoint = new Trigger(arcPID::atSetpoint);

  private final PIDController rotPID = new PIDController(0.02, 0.0, 0.00);
  private final Trigger atRotSetpoint = new Trigger(rotPID::atSetpoint);

  private final Trigger atPosition = atArcSetpoint.and(atRotSetpoint);

  /** Creates a new Claw. */
  public Claw() {
    rightMotor = new SparkMax(3, MotorType.kBrushless);
    leftMotor = new SparkMax(4, MotorType.kBrushless);

    gripMotor = new SparkMax(5, MotorType.kBrushless);

    leftConfig.idleMode(IdleMode.kCoast);
    rightConfig.idleMode(IdleMode.kCoast);

    leftConfig.smartCurrentLimit(5);
    rightConfig.smartCurrentLimit(5);

    rightConfig.inverted(true);
    leftConfig.inverted(false);


    leftMotor.configure(leftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    rightMotor.configure(rightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    

    arcThrift = new AnalogEncoder(2);

    rotationInput = new DigitalInput(0);

    detectStick = new DigitalInput(8);

    arcPID.setTolerance(ClawConstants.ARC_TOLERANCE);
    arcPID.setSetpoint(getArcDegrees());

    zeroRotation();
    rotPID.setTolerance(ClawConstants.ROT_TOLERANCE);
    rotPID.setSetpoint(0.0);

    // TODO: Uncomment when working
    // setDefaultCommand(
    //   new HomeWrist(this).andThen(runOnce(() -> setDefaultCommand(arcAndRotate())))
    // );
  }

  public double getArcDegrees()
  {
    return arcThrift.get() * 360.0 - Constants.ClawConstants.ENCODER_OFFSET;    
  }

  public void setGripSpeed(double speed)
  {
    gripMotor.set(speed);
  }

  public void setArcingSpeed(double speed)
  {
    leftMotor.set(speed);
    rightMotor.set(speed);
  }

  public double getArcSetpoint() {
    return arcPID.getSetpoint();
  }

  public Trigger atArcAngle() {
    return atArcSetpoint;
  }

  public void zeroRotation()
  {
    leftMotor.getEncoder().setPosition(0);
    rightMotor.getEncoder().setPosition(0);
  }

  public boolean isRotationHomed()
  {
    return !rotationInput.get();
  }

    /**
   * @return the atHome
   */
  public Trigger homed() {
    return atHome;
  }

  public double getLeftRotationDegrees()
  {
    return leftMotor.getEncoder().getPosition() * 2;

  }

  public double getRightRotationDegrees()
  {
    return rightMotor.getEncoder().getPosition() * 2;
  }

  public double getRotationDegrees() {
    return getLeftRotationDegrees() - getRotationDegrees();
  }

  public double getRotationSetpoint() {
    return rotPID.getSetpoint();
  }

  public Trigger atRotationAngle() {
    return atRotSetpoint;
  }

  public double getLeftRotationSpeed()
  {
    return leftMotor.getEncoder().getVelocity() / 11000.0;
  }


  public double getRightRotationSpeed()
  {
    return rightMotor.getEncoder().getVelocity() / 11000.0;
  }
  

  public void setRightSpeed(double speed)
  {
    rightMotor.set(MathUtil.clamp(speed, -ClawConstants.MAX_OUTPUT, ClawConstants.MAX_OUTPUT));
  }

  public void setLeftSpeed(double speed)
  {
    leftMotor.set(MathUtil.clamp(speed, -ClawConstants.MAX_OUTPUT, ClawConstants.MAX_OUTPUT));
  }


  public void setRotationSpeed(double speed)
  {
    leftMotor.set(MathUtil.clamp(speed, -ClawConstants.MAX_OUTPUT, ClawConstants.MAX_OUTPUT));
    rightMotor.set(MathUtil.clamp(-speed, -ClawConstants.MAX_OUTPUT, ClawConstants.MAX_OUTPUT));
  }

  private Command arcAndRotate() {
    return run(() -> {
      var arcOutput = MathUtil.clamp(
        arcPID.calculate(getArcDegrees()),
        -ClawConstants.MAX_OUTPUT * 0.5,
        ClawConstants.MAX_OUTPUT * 0.5
      );

      var rotOutput = MathUtil.clamp(
        arcPID.calculate(getRotationDegrees()),
        -ClawConstants.MAX_OUTPUT,
        ClawConstants.MAX_OUTPUT
      );

      leftMotor.set(arcOutput + rotOutput);
      rightMotor.set(arcOutput - rotOutput);
    });
  }

  public Command setPosition(double arcSetpoint, double rotSetpoint) {
    return runOnce(() -> {
      arcPID.setSetpoint(arcSetpoint);
      arcPID.reset();

      rotPID.setSetpoint(rotSetpoint);
      rotPID.reset();
    }).asProxy().withName("Arc: " + arcSetpoint + " Rotation: " + rotSetpoint)
      .andThen(arcAndRotate()).finallyDo(() -> setArcingSpeed(0)); // TODO: Remove when working
  }

  public Trigger atPosition() {
    return atPosition;
  }

  @Override
  public Command enableMotorBrakes(boolean enable) {
    var idleMode = enable ? IdleMode.kBrake : IdleMode.kCoast;

    return runOnce(
      () -> {
        leftMotor.configure(leftConfig.idleMode(idleMode), ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        rightMotor.configure(rightConfig.idleMode(idleMode), ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
      }
    ).ignoringDisable(true);
  }

  public boolean isSwitchTriggered()
  {
    return !detectStick.get();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Arc Degrees", getArcDegrees());
    SmartDashboard.putNumber("Arc Setpoint", getArcSetpoint());

    SmartDashboard.putNumber("Rotation Left Degrees", getLeftRotationDegrees());
    SmartDashboard.putNumber("Rotation Right Degrees", getRightRotationDegrees());
    SmartDashboard.putNumber("Rotation Degrees", getRotationDegrees());
    SmartDashboard.putNumber("Roation Setpoint", getRotationSetpoint());

    SmartDashboard.putBoolean("Is Homed", isRotationHomed());

    SmartDashboard.putNumber("Left Speed", getLeftRotationSpeed());
    SmartDashboard.putNumber("Right Speed", getRightRotationSpeed());

    SmartDashboard.putBoolean("In Position", isSwitchTriggered());
  }
}
