// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Claw;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class HomeWrist extends Command {
  Claw claw;

  double leftSpeed;
  double rightSpeed;

  PIDController leftPid = new PIDController(0.01, 0.0, 0.0);
  PIDController rightPid = new PIDController(0.01, 0.0, 0.0);

  /** Creates a new ArcingSpeed. */
  public HomeWrist(Claw claw) {
    // Use addRequirements() here to declare subsystem dependencies.

    this.claw = claw;

    addRequirements(claw);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {


    // leftSpeed = leftPid.calculate(claw.getLeftRotationSpeed(), -.4);
    claw.setLeftSpeed(-(.4));

    // rightSpeed = rightPid.calculate(claw.getRightRotationSpeed(), .4);
    
    claw.setRightSpeed((.4));



  }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    claw.setRotationSpeed(0);
    claw.zeroRotation();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return claw.isRotationHomed();
  }
}
