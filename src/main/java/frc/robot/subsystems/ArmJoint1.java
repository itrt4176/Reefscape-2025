// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmJoint1 extends SubsystemBase {

  AnalogEncoder joint1Encoder;

  AnalogEncoder joint2Encoder;

  TalonFX joint1Motor;

  TalonFX joint2Motor;
  /** Creates a new ArmSubsystem. */
  public ArmJoint1() {
    joint1Encoder = new AnalogEncoder(0);

    joint1Motor = new TalonFX(41);

  }

  public double getJointOnePosition()
  {
    return joint1Encoder.get() * 360.0;
  }

  
  public void setJoint1Speed(double speed)
  {
    joint1Motor.set(speed);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    SmartDashboard.putNumber("Arm Joint 1 Position", getJointOnePosition());
  }
}
