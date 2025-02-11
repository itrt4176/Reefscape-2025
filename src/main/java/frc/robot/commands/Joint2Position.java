// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.signals.PIDRefPIDErr_ClosedLoopModeValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmJoint1;
import frc.robot.subsystems.ArmJoint2;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Joint2Position extends Command {
  ArmJoint2 arm;

  double angle;

  double speed;

  PIDController pid = new PIDController(0.016, 0.0, 0.00);

  

  /** Creates a new Joint2Position. */
  public Joint2Position(ArmJoint2 arm, double angle) {
    this.angle = angle;

    this.arm = arm;


    addRequirements(arm);

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    speed = pid.calculate(arm.getJointTwoPosition(), angle);

    arm.setJoint2Speed(speed);
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
