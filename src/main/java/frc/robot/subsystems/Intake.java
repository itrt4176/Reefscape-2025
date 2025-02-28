// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ControlModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;



public class Intake extends SubsystemBase {
  AnalogInput sensor;

  AnalogEncoder encoder;

  TalonFX pivotMotor;

  TalonFX axelRotation;

  private double setpoint;

  /** Creates a new Intake. */
  public Intake() {

    // sensor = new AnalogInput(0);//placeholder

    encoder = new AnalogEncoder(3);

    pivotMotor = new TalonFX(30);//placeholder

    axelRotation = new TalonFX(31);//placeholder

    setpoint = IntakeConstants.STORE_ANGLE;
    setDefaultCommand(moveToAngle());
  }


  public double getPivotDegrees()
  {
    return encoder.get() * 360.0;
  }

  public void setSpeed(double speed)
  {
    axelRotation.set(speed);
  }

  public void setPivotSpeed(double speed)
  {
    pivotMotor.set(speed);
  }
  
  public void pivotStop()
  {
    pivotMotor.set(0);
  }

  private Command moveToAngle() {
    return run(() -> {
      var error = setpoint - getPivotDegrees();

      if (Math.abs(error) < 0.5) {
        error = 0.0;
      }

      setPivotSpeed(MathUtil.clamp(error * 0.0175, -0.1, 0.1));
    });
  }

  public void setPivotAngle(double angle) {
    setpoint = angle;
  }

  public boolean atAngleSetpoint() {
    return Math.abs(setpoint - getPivotDegrees()) <= 0.75;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Pivot Postition(Degrees)", getPivotDegrees());
  }
}