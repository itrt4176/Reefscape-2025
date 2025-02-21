// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
  /** Creates a new Climber. */

  private SparkMax winch;

  private SparkMaxConfig config;

  public Climber() {
    winch =  new SparkMax(51, MotorType.kBrushless);//placeholder

    config = new SparkMaxConfig();
    config.idleMode(IdleMode.kBrake);

    winch.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    
  }


  public void setWinchSpeed(double speed)
  {
    winch.set(speed);
  }

  /** 
  * Returns the rotations per second of winch motor.
  */
  public double getWinchSpeed()
  {
    return winch.getEncoder().getVelocity();
  }

  public double getWinchPosition(){
    return winch.getEncoder().getPosition();
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    SmartDashboard.putNumber("Winch Speed", getWinchSpeed());
  }
}
