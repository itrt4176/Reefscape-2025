// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Minute;
import static edu.wpi.first.units.Units.Rotations;

import java.util.EnumMap;
import java.util.Map;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ClawConstants;

public class Claw extends SubsystemBase {
  
  /** 
   * The left motor controller controlling the wrist
   * */ 
  SparkMax leftMotor;

  /** 
   * The right motor controller controlling the wrist
   * */ 
  private SparkMax rightMotor;

  private AnalogEncoder arcThrift;

  private SparkMaxConfig leftConfig = new SparkMaxConfig();
  private SparkMaxConfig rightConfig = new SparkMaxConfig();

  private DigitalInput rotationInput;


  private MutAngle angle;
  private MutAngle goal;
  private MutAngularVelocity velocity;

  private ProfiledPIDController pidControl;

  private ArmFeedforward ff;

  private Trigger atGoal;
   

  /** Creates a new Claw. */
  public Claw() {
    rightMotor = new SparkMax(3, MotorType.kBrushless);
    leftMotor = new SparkMax(4, MotorType.kBrushless);

    leftConfig.idleMode(IdleMode.kBrake);
    rightConfig.idleMode(IdleMode.kBrake);

    rightConfig.inverted(true);

    leftConfig.openLoopRampRate(0);
    rightConfig.openLoopRampRate(0);


    leftMotor.configure(leftConfig, null, PersistMode.kPersistParameters);
    rightMotor.configure(rightConfig, null, PersistMode.kPersistParameters);
    

    arcThrift = new AnalogEncoder(ClawConstants.CLAW_ENCODER_CHANNEL);

    rotationInput = new DigitalInput(0);

    goal = Degrees.mutable(ClawConstants.STOW);

    angle = Degrees.mutable(arcThrift.get() * 360);
    velocity = DegreesPerSecond.mutable(0);


    pidControl = new ProfiledPIDController(
              0.012, 
              0.0, 
              0.0, 
              new TrapezoidProfile.Constraints(
                  90,  
                  180),
                  0.02);


    pidControl.setTolerance(0.5);

    ff = new ArmFeedforward(
            0.01, 
            0.0, 
            12.0,
            1.305,
            0.02);
          
    pidControl.setGoal(goal.in(Degrees));
    pidControl.reset(angle.in(Degrees), velocity.in(DegreesPerSecond));

    setDefaultCommand(run(this::setArcingSpeed));

    atGoal = new Trigger(pidControl::atGoal);
      
  }

  public double getArcDegrees()
  {
    return arcThrift.get() * 360.0;

    
  }


  public Command setNewAngle(double position){
    return runOnce(() -> {
        goal.mut_replace(position, Degrees);
        pidControl.setGoal(goal.in(Degrees));
        pidControl.reset(angle.in(Degrees), velocity.in(DegreesPerSecond));

    });
  }

  public void setArcingSpeed()
  {
    double volts = pidControl.calculate(getArcDegrees(), goal.magnitude())
              + ff.calculate(pidControl.getSetpoint().position, pidControl.getSetpoint().velocity);

    leftMotor.setVoltage(volts);
    rightMotor.setVoltage(volts);
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

  public double getLeftRotationDegrees()
  {
    return leftMotor.getEncoder().getPosition();

  }

  public double getRightRotationDegrees()
  {
    return rightMotor.getEncoder().getPosition();
  }

  public double getLeftRotationSpeed()
  {
    return leftMotor.getEncoder().getVelocity() / 11000.0;
  }


  public double getRotationSpeed()
  {
    velocity.mut_replace(rightMotor.getEncoder().getVelocity(), Rotations.per(Minute));
    return velocity.in(DegreesPerSecond);
  }

  public void setRightSpeed(double speed)
  {
    rightMotor.set(speed);
  }

  public void setLeftSpeed(double speed)
  {
    leftMotor.set(speed);
  }


  public void setRotationSpeed(double speed)
  {
    leftMotor.set(speed);
    rightMotor.set(-speed);
  }



  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    SmartDashboard.putNumber("Arc Degrees", getArcDegrees());
    SmartDashboard.putNumber("Rotation Left Degrees", getLeftRotationDegrees());
    SmartDashboard.putNumber("Rotation Right Degrees", getRightRotationDegrees());
    SmartDashboard.putBoolean("Is Homed", isRotationHomed());
  }
}
