// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmJoint2 extends SubsystemBase {

  SparkMax joint2Motor = new SparkMax(17, MotorType.kBrushless);

  AnalogEncoder joint2Encoder = new AnalogEncoder(2);
  /** Creates a new ArmJoint2. */
  public ArmJoint2() {

  }


  
  public void setJoint2Speed(double speed)
  {
    joint2Motor.set(speed);
  }

  public double getJointTwoPosition()
  {
    return joint2Encoder.get() * 360.0;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    SmartDashboard.putNumber("Arm Joint 2 Position", getJointTwoPosition());

  }
}
