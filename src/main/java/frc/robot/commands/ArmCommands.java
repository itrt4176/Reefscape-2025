package frc.robot.commands;

import java.util.function.DoubleSupplier;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.ArmJoint;
import frc.robot.subsystems.ArmJoint.Position;

public class ArmCommands {
  private final ArmJoint joint1;
  private final ArmJoint joint2;
  private final Trigger atGoal;

  public ArmCommands(ArmJoint joint1, ArmJoint joint2) {
    this.joint1 = joint1;
    this.joint2 = joint2;
    atGoal = joint1.atGoal().and(joint2.atGoal());

    // for (Position position : Position.values()) {
    //   NamedCommands.registerCommand(position.toString(), setPosition(position));
    // }
  }

  public Command setPosition(Position position) {
    return joint1.setPosition(position)
      .alongWith(joint2.setPosition(position));
  }

  public Command adjustOffset(DoubleSupplier joint1Adjuster, DoubleSupplier joint2Adjuster) {
    return joint1.adjustGoal(joint1Adjuster)
      .alongWith(joint2.adjustGoal(joint2Adjuster));
  }

  public Trigger atGoal() {
    return atGoal;
  }
}
