// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.ShoulderJointConstants;
import frc.robot.Constants.ElbowJointConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.ArmJoint;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

  private final ArmJoint shoulderJoint = new ArmJoint(
    ShoulderJointConstants.motorPort,
    ShoulderJointConstants.encoderPort,
    ShoulderJointConstants.pidConfig,
    ShoulderJointConstants.angleMap,
    "Shoulder Joint"
  );

  // private final ArmJoint elbowJoint = new ArmJoint(
  //   ElbowJointConstants.motorPort,
  //   ElbowJointConstants.encoderPort,
  //   ElbowJointConstants.pidConfig,
  //   ElbowJointConstants.angleMap,
  //   "Elbow Joint"
  // );

  private final Command joint1TwoHundred = shoulderJoint.setPosition(ArmJoint.Position.LEVEL_ONE);

  private final Command fullyErect1 = shoulderJoint.setPosition(ArmJoint.Position.LEVEL_FOUR);

  // private final Command fullyErect2 = elbowJoint.setPosition(ArmJoint.Position.LEVEL_FOUR);

  private final Command intakeJoint1 = shoulderJoint.setPosition(ArmJoint.Position.INTAKE);

  // private final Command intakeJoint2 = elbowJoint.setPosition(ArmJoint.Position.INTAKE);

  private final Command stow1 = shoulderJoint.setPosition(ArmJoint.Position.STOW);

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Disable the arm joints for tuning
    shoulderJoint.setEnabled(false);
    // elbowJoint.setEnabled(false);

    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    new Trigger(m_exampleSubsystem::exampleCondition)
        .onTrue(new ExampleCommand(m_exampleSubsystem));

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    // m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());

    // m_driverController.a().whileTrue(new InstantCommand(() -> arm.setJoint2Speed(0.3)));

    // m_driverController.y().whileTrue(new InstantCommand(() -> arm.setJoint2Speed(-0.3)));

    // m_driverController.rightBumper().whileTrue(new InstantCommand(() -> arm.setJoint1Speed(.05)));
    // m_driverController.leftBumper().whileTrue(new InstantCommand(() -> arm.setJoint1Speed(-0.05)));

    // m_driverController.b().onTrue(new InstantCommand(() -> arm.setJoint1Speed(0)));
    // m_driverController.x().onTrue(new InstantCommand(() -> arm.setJoint2Speed(0)));




    // m_driverController.a().onTrue(joint1TwoHundred);
    // m_driverController.b().onTrue(new InstantCommand(() -> arm.setJoint1Speed(0)));


    // m_driverController.y().onTrue(intakeJoint1);

    // m_driverController.a().onTrue(intakeJoint2);

    // Shoulder joint sysid routines
    // Hold down each button to perform its routine
    m_driverController.y().whileTrue(shoulderJoint.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    m_driverController.a().whileTrue(shoulderJoint.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    m_driverController.b().whileTrue(shoulderJoint.sysIdDynamic(SysIdRoutine.Direction.kForward));
    m_driverController.x().whileTrue(shoulderJoint.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    // Elbow joint sysid routines
    // Hold down each button to perform its routine
    // m_driverController.y().whileTrue(elbowJoint.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    // m_driverController.a().whileTrue(elbowJoint.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    // m_driverController.b().whileTrue(elbowJoint.sysIdDynamic(SysIdRoutine.Direction.kForward));
    // m_driverController.x().whileTrue(elbowJoint.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    m_driverController.rightBumper().onTrue(fullyErect1);
    // m_driverController.leftBumper().onTrue(fullyErect2);

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return Autos.exampleAuto(m_exampleSubsystem);
  }
}
