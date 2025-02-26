// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.units.Unit;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;

public class Claw extends SubsystemBase {
  
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

  private final Trigger atHome = new Trigger(this::isRotationHomed);

  /** Creates a new Claw. */
  public Claw() {
    rightMotor = new SparkMax(3, MotorType.kBrushless);
    leftMotor = new SparkMax(4, MotorType.kBrushless);

    gripMotor = new SparkMax(5, MotorType.kBrushless);

    leftConfig.idleMode(IdleMode.kBrake);
    rightConfig.idleMode(IdleMode.kBrake);

    rightConfig.inverted(true);


    leftMotor.configure(leftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    rightMotor.configure(rightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    

    arcThrift = new AnalogEncoder(2);

    rotationInput = new DigitalInput(0);

    zeroRotation();
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

    SmartDashboard.putNumber("Left Speed", getLeftRotationSpeed());
    SmartDashboard.putNumber("Right Speed", getRightRotationSpeed());
  }
}
