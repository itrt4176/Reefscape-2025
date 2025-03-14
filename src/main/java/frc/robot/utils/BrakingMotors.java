package frc.robot.utils;

import edu.wpi.first.wpilibj2.command.Command;

public interface BrakingMotors {
  public void setMotorBrakes(boolean enable);
  public Command motorBrakes(boolean enable);
}
