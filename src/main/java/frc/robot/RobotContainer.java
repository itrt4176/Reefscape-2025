// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.ShoulderJointConstants;
import frc.robot.Constants.ElbowJointConstants;

import static edu.wpi.first.units.Units.Degrees;

import java.io.File;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.ArcingSpeed;
import frc.robot.commands.ArmCommands;
import frc.robot.commands.ClawSetArcAngle;
import frc.robot.commands.CrazyShit;
import frc.robot.commands.HomeWrist;
import frc.robot.commands.RotationSetpoint;
import frc.robot.commands.RotationSpeed;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.ArmJoint;
import frc.robot.subsystems.ArmJoint.Position;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import swervelib.SwerveInputStream;
import frc.robot.utils.CommandTigerPad;
import frc.robot.utils.TigerPad.LEDMode;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  private final Claw claw = new Claw();

  private final ArcingSpeed arcing = new ArcingSpeed(claw, 0.05);

  private final ArcingSpeed reverseArc = new ArcingSpeed(claw, -0.05);
  private final RotationSpeed rotate = new RotationSpeed(claw, .3);

  private final Command levelFour = new ClawSetArcAngle(claw, 50)
      .andThen(new RotationSetpoint(claw, 0));

  private final ClawSetArcAngle intakeClaw = new ClawSetArcAngle(claw, 50);

  // private final ClawSetArcAngle levelOneClaw = new ClawSetArcAngle(claw, 160);

  // private final ClawSetArcAngle levelTwoClaw = new ClawSetArcAngle(claw, 206.0);


  // private final ClawSetArcAngle levelThreeClaw = new ClawSetArcAngle(claw, 225.0);

  private final RotationSetpoint ninetyRot = new RotationSetpoint(claw, 90);

  private final HomeWrist homeWrist = new HomeWrist(claw);

  private final CommandXboxController driverController = new CommandXboxController(
      OperatorConstants.DRIVE_CONTROLLER_PORT);

  private final CommandTigerPad armControlPanel = CommandTigerPad.getInstance(
      OperatorConstants.ARM_CONTROL_PANEL_PORT);

  private final ArmJoint shoulderJoint = new ArmJoint(
    ShoulderJointConstants.motorPort,
    ShoulderJointConstants.encoderPort,
    ShoulderJointConstants.encoderOffset,
    ShoulderJointConstants.pidConfig,
    ShoulderJointConstants.angleMap,
    "Shoulder Joint",
    false
  );

  private final ArmJoint elbowJoint = new ArmJoint(
    ElbowJointConstants.motorPort,
    ElbowJointConstants.encoderPort,
    ElbowJointConstants.encoderOffset,
    () -> shoulderJoint.getAngle().magnitude(),
    ElbowJointConstants.pidConfig,
    ElbowJointConstants.angleMap,
    "Elbow Joint",
    false
  );

  private final ArmCommands armCommands = new ArmCommands(shoulderJoint, elbowJoint);

  private final SwerveSubsystem drivebase = new SwerveSubsystem(
    new File(Filesystem.getDeployDirectory(), "swerve/neo")
  );


  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Disable the arm joints for tuning
    shoulderJoint.setEnabled(true);
    elbowJoint.setEnabled(true);

    // Configure the trigger bindings
    configureBindings();
    DriverStation.silenceJoystickConnectionWarning(true);

    //Apply inversion for inversion later
    Command joystickDrive = drivebase.driveCommand(
                    () -> MathUtil.applyDeadband(driverController.getLeftY(), 0.1), 
                    () -> MathUtil.applyDeadband(driverController.getLeftX(), 0.1), 
                    () -> MathUtil.applyDeadband(-driverController.getRightX(), 0.1));


    drivebase.setDefaultCommand(joystickDrive);
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the {@link Trigger#Trigger(java.util.function.BooleanSupplier)}
   * constructor with an arbitrary predicate, or via the named factories in
   * {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s
   * subclasses for {@link CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    driverController.rightBumper().whileTrue(new StartEndCommand(() -> claw.setGripSpeed(0.30), () -> claw.setGripSpeed(0), claw));
    driverController.leftBumper().whileTrue(new StartEndCommand(() -> claw.setGripSpeed(-0.30), () -> claw.setGripSpeed(0), claw));


    driverController.a().onTrue(
      shoulderJoint.setPosition(Position.LEVEL_ONE)
        .alongWith(elbowJoint.setPosition(Position.LEVEL_ONE))
    );


    driverController.b().onTrue(
      elbowJoint.setPosition(Position.INTAKE).asProxy()
        .andThen(Commands.waitUntil(() -> elbowJoint.getAngle().in(Degrees) >= 45))
        .andThen(shoulderJoint.setPosition(Position.INTAKE))
        .alongWith(ninetyRot.andThen(intakeClaw))
    );

    driverController.x().onTrue(
      shoulderJoint.setPosition(Position.LEVEL_TWO)
        .alongWith(elbowJoint.setPosition(Position.LEVEL_TWO))
    );

    driverController.y().onTrue(
      shoulderJoint.setPosition(Position.LEVEL_FOUR)
        .alongWith(elbowJoint.setPosition(Position.LEVEL_FOUR), levelFour)
    );

    driverController.start().onTrue(homeWrist);

    claw.homed().onTrue(Commands.runOnce(claw::zeroRotation));
  }

  private void configureSysIdBindings() {
    // Shoulder joint sysid routines
    // Hold down each button to perform its routine
    driverController.y().whileTrue(shoulderJoint.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    driverController.a().whileTrue(shoulderJoint.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    driverController.b().whileTrue(shoulderJoint.sysIdDynamic(SysIdRoutine.Direction.kForward));
    driverController.x().whileTrue(shoulderJoint.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    // Elbow joint sysid routines
    // Hold down each button to perform its routine
    driverController.y().whileTrue(elbowJoint.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    driverController.a().whileTrue(elbowJoint.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    driverController.b().whileTrue(elbowJoint.sysIdDynamic(SysIdRoutine.Direction.kForward));
    driverController.x().whileTrue(elbowJoint.sysIdDynamic(SysIdRoutine.Direction.kReverse));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return null;
    // return elbowJoint.setPosition(ArmJoint.Position.STOW);
  }

  public void setMotorBrake(boolean brake) { drivebase.setMotorBrake(brake); }
}
