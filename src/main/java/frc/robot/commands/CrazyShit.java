// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ClawConstants;
import frc.robot.subsystems.Claw;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class CrazyShit extends Command {

  private Claw claw;

  private double rotSetpoint; 
  private double arcSetpoint;

  private double rotAngle;
  private double arcAngle;

  private double rotSpeed;

  private double arcSpeed;

  private boolean rotationTime;

  private PIDController rotPID = new PIDController(0.02, 0.0, 0.00);

  private PIDController arcPID = new PIDController(0.014, 0.000, 0.0);


  /** Creates a new CrazyShit. */
  public CrazyShit(Claw claw, double rotSetpoint, double arcSetpoint) {
    // Use addRequirements() here to declare subsystem dependencies.

    this.claw = claw;

    this.rotSetpoint = rotSetpoint;

    this.arcSetpoint = arcSetpoint;

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
      rotationTime = false;
      rotAngle = claw.getLeftRotationDegrees() + -claw.getRightRotationDegrees();

      if (rotAtSetpoint()) return;

      rotSpeed = rotPID.calculate(rotAngle, rotSetpoint);
      claw.setRotationSpeed(rotSpeed);
    }
    else
    {
      arcSpeed = MathUtil.clamp(arcPID.calculate(claw.getArcDegrees(), arcSetpoint), -ClawConstants.MAX_OUTPUT * 0.5, ClawConstants.MAX_OUTPUT * 0.5);

      rotationTime = true;

      if (arcAtSetpoint()) return;
      
      claw.setArcingSpeed(arcSpeed);
    }
    

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    claw.setArcingSpeed(0.0);
  }

  private boolean rotAtSetpoint() {
    return rotSpeed < 0.02 && (Math.abs(rotAngle) - rotSetpoint) < ClawConstants.ROT_TOLERANCE;
  }

  private boolean arcAtSetpoint() {
    return arcSpeed < 0.01 && (Math.abs(claw.getArcDegrees() - arcAngle) < ClawConstants.ARC_TOLERANCE);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return arcAtSetpoint() && rotAtSetpoint();
  }
}
