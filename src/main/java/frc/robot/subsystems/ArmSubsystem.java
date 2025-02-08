// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase {

  AnalogEncoder joint1Encoder;

  AnalogEncoder joint2Encoder;

  TalonFX joint1Motor;

  TalonFX joint2Motor;
  /** Creates a new ArmSubsystem. */
  public ArmSubsystem() {
    joint1Encoder = new AnalogEncoder(3);
    // joint2Encoder = new AnalogEncoder(2);

    joint1Motor = new TalonFX(21);
    joint2Motor = new TalonFX(17);

  }

  public double getJointOnePosition()
  {
    return joint1Encoder.get();
  }

  // public double getJointTwoPosition()
  // {
  //   return joint2Encoder.get();
  // }

  public void setJoint1Speed(double speed)
  {
    joint1Motor.set(speed);
  }

  public void setJoint2Speed(double speed)
  {
    joint2Motor.set(speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    SmartDashboard.putNumber("Arm Joint 1 Position", getJointOnePosition());
    // SmartDashboard.putNumber("Arm Joint 2 Position", getJointTwoPosition());
  }
}
