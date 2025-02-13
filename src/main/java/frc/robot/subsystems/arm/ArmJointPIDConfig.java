package frc.robot.subsystems.arm;

public record ArmJointPIDConfig(
    double p,
    double i,
    double d,
    double maxVelocity,
    double maxAcceleration,
    double tolerance,
    double s,
    double g,
    double v,
    double loopTime
) {}
