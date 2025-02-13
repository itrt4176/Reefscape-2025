package frc.robot.subsystems.arm;

import java.util.function.DoubleSupplier;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;

public interface ArmJointSubsystem {

    Angle getAngle();

    Angle getGoal();

    Angle getOffset();

    Command setPosition(ArmPosition position);

    Command adjustOffset(DoubleSupplier offsetSupplier);

}