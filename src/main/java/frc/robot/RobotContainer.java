// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.ShoulderJointConstants;
import frc.robot.Constants.ElbowJointConstants;
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
import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.ArmJoint;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.ArmJoint.Position;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import swervelib.SwerveInputStream;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

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

  private Climber climber = new Climber();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController = new CommandXboxController(
      OperatorConstants.kDriverControllerPort);

  private final SwerveSubsystem drivebase = new SwerveSubsystem(
      new File(Filesystem.getDeployDirectory(), "swerve/neo"));

//   SwerveInputStream driveAngularVelocity = SwerveInputStream
//       .of(drivebase.getSwerveDrive(), () -> m_driverController.getLeftY() * -1,
//           () -> m_driverController.getLeftX() * -1)
//       .withControllerRotationAxis(m_driverController::getRightX)
//       .deadband(OperatorConstants.DEADBAND).scaleTranslation(0.8)
//       .allianceRelativeControl(true);

  /**
   * Clone's the angular velocity input stream and converts it to a
   * fieldRelative input stream.
   */
//   SwerveInputStream driveDirectAngle = driveAngularVelocity.copy()
//       .withControllerHeadingAxis(m_driverController::getRightX,
//           m_driverController::getRightY)
//       .headingWhile(true);

  /**
   * Clone's the angular velocity input stream and converts it to a
   * robotRelative input stream.
   */
//   SwerveInputStream driveRobotOriented = driveAngularVelocity.copy()
//       .robotRelative(true).allianceRelativeControl(false);

  SwerveInputStream driveAngularVelocityKeyboard = SwerveInputStream
      .of(drivebase.getSwerveDrive(), () -> -m_driverController.getLeftY(),
          () -> -m_driverController.getLeftX())
      .withControllerRotationAxis(() -> m_driverController.getRawAxis(2))
      .deadband(OperatorConstants.DEADBAND).scaleTranslation(0.8)
      .allianceRelativeControl(true);
  // Derive the heading axis with math!
  SwerveInputStream driveDirectAngleKeyboard = driveAngularVelocityKeyboard
      .copy()
      .withControllerHeadingAxis(
          () -> Math.sin(m_driverController.getRawAxis(2) * Math.PI)
              * (Math.PI * 2),
          () -> Math.cos(m_driverController.getRawAxis(2) * Math.PI)
              * (Math.PI * 2))
      .headingWhile(true);

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
                    () -> MathUtil.applyDeadband(m_driverController.getLeftY(), 0.1), 
                    () -> MathUtil.applyDeadband(0.0, 0.1), 
                    () -> MathUtil.applyDeadband(0.0, 0.1));


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
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    new Trigger(m_exampleSubsystem::exampleCondition)
        .onTrue(new ExampleCommand(m_exampleSubsystem));

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is
    // pressed,
    // cancelling on release.
    
    // Shoulder joint sysid routines
    // Hold down each button to perform its routine
    // m_driverController.y().whileTrue(shoulderJoint.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    // m_driverController.a().whileTrue(shoulderJoint.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    // m_driverController.b().whileTrue(shoulderJoint.sysIdDynamic(SysIdRoutine.Direction.kForward));
    // m_driverController.x().whileTrue(shoulderJoint.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    // Elbow joint sysid routines
    // Hold down each button to perform its routine
    // m_driverController.y().whileTrue(elbowJoint.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    // m_driverController.a().whileTrue(elbowJoint.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    // m_driverController.b().whileTrue(elbowJoint.sysIdDynamic(SysIdRoutine.Direction.kForward));
    // m_driverController.x().whileTrue(elbowJoint.sysIdDynamic(SysIdRoutine.Direction.kReverse));


  //   m_driverController.a().onTrue(
  //     shoulderJoint.setPosition(Position.LEVEL_ONE)
  //       .alongWith(elbowJoint.setPosition(Position.LEVEL_ONE))
  //   );

  //   m_driverController.b().onTrue(
  //     shoulderJoint.setPosition(Position.INTAKE)
  //       .alongWith(elbowJoint.setPosition(Position.INTAKE))
  //   );

  //   m_driverController.x().onTrue(
  //     shoulderJoint.setPosition(Position.LEVEL_THREE)
  //       .alongWith(elbowJoint.setPosition(Position.LEVEL_THREE))
  //   );

  //   m_driverController.y().onTrue(
  //     shoulderJoint.setPosition(Position.LEVEL_FOUR)
  //       .alongWith(elbowJoint.setPosition(Position.LEVEL_FOUR))
  //   );

    m_driverController.a().whileTrue(new StartEndCommand(() -> climber.setWinchSpeed(0.1), () -> climber.setWinchSpeed(0), climber));

  }

    

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return Autos.exampleAuto(m_exampleSubsystem);
    // return elbowJoint.setPosition(ArmJoint.Position.STOW);
  }

  public void setMotorBrake(boolean brake) { drivebase.setMotorBrake(brake); }
}
