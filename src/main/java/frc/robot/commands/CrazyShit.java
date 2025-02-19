// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Claw;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class CrazyShit extends Command {

  private Claw claw;

  double angleRot; 
  double angleArc;

  double leftSpeed;
  double rightSpeed;

  double arcSpeed;

  boolean rotationTime;

  PIDController leftpid = new PIDController(0.02, 0.0, 0.00);
  PIDController rightpid = new PIDController(0.02, 0.0, 0.00);

  private PIDController pid = new PIDController(0.012, 0.000, 0.0);


  /** Creates a new CrazyShit. */
  public CrazyShit(Claw claw, double angleRot, double angleArc) {
    // Use addRequirements() here to declare subsystem dependencies.

    this.claw = claw;
    this.angleRot = angleRot;

    this.angleArc = angleArc;


    addRequirements(claw);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    rotationTime = true;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {


    if(rotationTime)
    {
      double setpoint = (angleRot/2.0);

    leftSpeed = leftpid.calculate(claw.getLeftRotationDegrees(), setpoint);
    claw.setLeftSpeed(leftSpeed);


    rightSpeed = rightpid.calculate(claw.getRightRotationDegrees(), -setpoint);
    claw.setRightSpeed(rightSpeed);

    rotationTime = false;
    }
    else
    {
      arcSpeed = pid.calculate(claw.getArcDegrees(), angleArc);

      claw.setArcingSpeed(-arcSpeed);

      rotationTime = true;
    }
    

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
