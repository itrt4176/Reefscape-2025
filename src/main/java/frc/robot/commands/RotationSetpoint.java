// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ClawConstants;
import frc.robot.subsystems.Claw;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class RotationSetpoint extends Command {

  Claw claw;

  double angle; 

  double leftSpeed;
  double rightSpeed;

  PIDController leftpid = new PIDController(0.02, 0.0, 0.00);
  PIDController rightpid = new PIDController(0.02, 0.0, 0.00);

  
  /** Creates a new RotationSetpoint. */
  public RotationSetpoint(Claw claw, double angle) {
    // Use addRequirements() here to declare subsystem dependencies.

    this.claw = claw;
    this.angle = angle;

    addRequirements(claw);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    leftSpeed = leftpid.calculate(claw.getLeftRotationDegrees() + -claw.getRightRotationDegrees(), angle);
    claw.setRotationSpeed(leftSpeed);
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    leftSpeed = 0;
    rightSpeed = 0;

    claw.setLeftSpeed(leftSpeed);
    claw.setRightSpeed(rightSpeed);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return ((leftSpeed + rightSpeed) < 0.02 && (Math.abs((claw.getLeftRotationDegrees()-claw.getRightRotationDegrees()) - angle) < 1.0));

  }
}
