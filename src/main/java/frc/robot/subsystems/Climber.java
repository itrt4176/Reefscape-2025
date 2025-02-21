// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
  /** Creates a new Climber. */

  private TalonFX winch;

  public Climber() {
    winch =  new TalonFX(0);//placeholder

    winch.setNeutralMode(NeutralModeValue.Brake);
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
    return winch.getVelocity().getValueAsDouble();
  }

  public double getWinchPosition(){
    return winch.getPosition().getValueAsDouble();
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    SmartDashboard.putNumber("Winch Speed", getWinchSpeed());
  }
}
