// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ControlModeValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {

  TalonFX armJoint1;

  TalonFX armJoint2;

  TalonFXConfiguration configJoint1 = new TalonFXConfiguration();
  TalonFXConfiguration configJoint2 = new TalonFXConfiguration();

  
  final MotionMagicVoltage request = new MotionMagicVoltage(0);

  /** Creates a new Arm. */
  public Arm() {
    armJoint1 = new TalonFX(21);//placeholder



    //set slot 0 gains?
    var joint1Configs = configJoint1.Slot0; 
    joint1Configs.kS = .25; //placeholder --> used to overcome static friction
    joint1Configs.kV = 0.12; // placeholder --> voltage output for 1 rps
    joint1Configs.kA = 0.01; //placeholder --> voltage
    joint1Configs.kP = 8.0; //placeholder for position error (should be Voltage/rotation error)
    joint1Configs.kI = 0.0; //placeholder 
    joint1Configs.kD = 0.1; //placeholder (Voltage/rps)


    

    var joint1MotionMagicConfig = configJoint1.MotionMagic;
    joint1MotionMagicConfig.MotionMagicCruiseVelocity = 80.0;//target velocitoy f .5 rps (placeholder)
    joint1MotionMagicConfig.MotionMagicAcceleration =  160.0; //double to get half second time (placeholder)
    joint1MotionMagicConfig.MotionMagicJerk = 1600.0;//target jerk (placeholder)

  
    armJoint1.getConfigurator().apply(joint1Configs);

    

    
  }

  public void setPosition(double position)
  {
    armJoint1.setControl(request.withPosition(position));
  }

  public double getPosition()
  {
    return armJoint1.getPosition().getValueAsDouble();
  }

  public void setSpeed(double speed)
  {
    armJoint1.set(speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    SmartDashboard.putNumber("Position", getPosition());
  }
}
