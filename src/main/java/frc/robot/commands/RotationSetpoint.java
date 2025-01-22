// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Claw;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class RotationSetpoint extends Command {

  Claw claw;

  double angle;

  double speed;

  PIDController pid = new PIDController(0.001, 0.0, 0.0);

  
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
    double setpoint = angle / 0.035;

    speed = pid.calculate(claw.getLeftRotationDegrees(), setpoint);

    claw.setRotationSpeed(speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(speed <= 0.01)
    {
      return true;
    }
    return false;
  }
}
