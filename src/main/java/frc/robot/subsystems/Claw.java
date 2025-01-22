// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Claw extends SubsystemBase {
  
  /** 
   * The left motor controller controlling the wrist
   * */ 
  SparkMax leftMotor;

  /** 
   * The right motor controller controlling the wrist
   * */ 
  SparkMax rightMotor;

  AnalogEncoder arcThrift;

  DigitalInput rotationInput;

  /** Creates a new Claw. */
  public Claw() {
    rightMotor = new SparkMax(3, MotorType.kBrushless);
    leftMotor = new SparkMax(4, MotorType.kBrushless);

    arcThrift = new AnalogEncoder(0);

    rotationInput = new DigitalInput(0);
  }

  public double getArcDegrees()
  {
    return arcThrift.get() * 360.0;

    
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

  public double getLeftRotationDegrees()
  {
    return leftMotor.getEncoder().getPosition();
    

  }

  public double getRightRotationDegrees()
  {
    return rightMotor.getEncoder().getPosition();
  }

  public void setRotationSpeed(double speed)
  {
    if(speed == 0)
    {
      leftMotor.set(speed);
      rightMotor.set(speed);
    }
    else
    {
      leftMotor.set(speed);
      rightMotor.set(-(speed - 0.05));
    }
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
