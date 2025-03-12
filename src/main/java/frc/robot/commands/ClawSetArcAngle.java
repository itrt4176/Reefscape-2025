// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ClawConstants;
import frc.robot.subsystems.Claw;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ClawSetArcAngle extends Command {

  private Claw claw;

  private DoubleSupplier angleSupplier;

  private double speed;

  private PIDController pid = new PIDController(0.02, 0.000, 0.0); //test for vals
  
  /** Creates a new ArmSetArcAngle. */
  public ClawSetArcAngle(Claw claw, DoubleSupplier angleSupplier) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.claw = claw;
    this.angleSupplier = angleSupplier;

    addRequirements(claw);
  }

  public ClawSetArcAngle(Claw claw, double angle) {
    this(claw, () -> angle);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    speed = MathUtil.clamp(pid.calculate(claw.getArcDegrees(), angleSupplier.getAsDouble()), ClawConstants.MAX_OUTPUT * -.5, ClawConstants.MAX_OUTPUT *.5);

    claw.setArcingSpeed(speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    speed = 0;

    claw.setArcingSpeed(speed);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (speed < 0.01 && (Math.abs(claw.getArcDegrees() - angleSupplier.getAsDouble()) < 0.8));
    
  }
}
