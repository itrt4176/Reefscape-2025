package frc.robot.commands;

import java.util.function.DoubleSupplier;
import java.util.function.Function;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.ArmJoint;
import frc.robot.subsystems.ArmJoint.Position;
import frc.robot.utils.TigerPad.LEDMode;

public class ArmCommands {
  private final ArmJoint shoulderJoint;
  private final ArmJoint elbowJoint;
  private final Trigger atGoal;

  public ArmCommands(ArmJoint shoulderJoint, ArmJoint elbowJoint) {
    this.shoulderJoint = shoulderJoint;
    this.elbowJoint = elbowJoint;
    atGoal = shoulderJoint.atGoal().and(elbowJoint.atGoal());
  }

  public Command setPosition(Position position) {
    return shoulderJoint.setPosition(position)
      .alongWith(elbowJoint.setPosition(position));
  }

  public Command setPosition(Position position, Function<LEDMode, Command> setAllLEDs, Function<LEDMode, Command> setLED) {
    return setAllLEDs.apply(LEDMode.Off)
      .andThen(setPosition(position).asProxy())
      .andThen(setLED.apply(LEDMode.Blink))
      .andThen(Commands.waitUntil(atGoal))
      .andThen(setLED.apply(LEDMode.On));
  }

  public Command adjustOffset(DoubleSupplier shoulderJointAdjuster, DoubleSupplier elbowJointAdjuster) {
    return shoulderJoint.adjustGoal(shoulderJointAdjuster)
      .alongWith(elbowJoint.adjustGoal(elbowJointAdjuster));
  }

  public Command directDrive(DoubleSupplier shoulderJointOutput, DoubleSupplier elbowJointOutput) {
    return shoulderJoint.directDrive(shoulderJointOutput)
      .alongWith(elbowJoint.directDrive(elbowJointOutput));
  }

  public Trigger atGoal() {
    return atGoal;
  }
}
