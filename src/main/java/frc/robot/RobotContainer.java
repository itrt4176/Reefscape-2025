// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.ArcingSpeed;
import frc.robot.commands.ClawSetArcAngle;
import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.HomeWrist;
import frc.robot.commands.RotationSetpoint;
import frc.robot.commands.RotationSpeed;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

  private final Claw claw = new Claw();

  private final ArcingSpeed arcing = new ArcingSpeed(claw, 0.1);

  private final ArcingSpeed reverseArc = new ArcingSpeed(claw, -0.1);

  private final RotationSpeed rotate = new RotationSpeed(claw, .3);

  private final ClawSetArcAngle twoThirty = new ClawSetArcAngle(claw, 205);

  private final RotationSetpoint ninetyRot = new RotationSetpoint(claw, 90);

  private final HomeWrist homeWrist = new HomeWrist(claw);

  // private final SequentialCommandGroup homing = new SequentialCommandGroup(twoThirty, homeWrist);

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
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
    // driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());

    // driverController.x().onTrue(new InstantCommand(() -> claw.setRotationSpeed(.05)));
    // driverController.y().onTrue(new InstantCommand(() -> claw.setRotationSpeed(-.05)));
    // driverController.b().onTrue(new InstantCommand(() -> claw.setArcingSpeed(0)));

    // driverController.x().onTrue(fourtyFive);

    // driverController.rightBumper().onTrue(new InstantCommand(() -> claw.setArcingSpeed(.05)));
    // driverController.leftBumper().onTrue(new InstantCommand(() -> claw.setArcingSpeed(-.05)));

  // driverController.y().onTrue(new InstantCommand(() -> claw.setRotationSpeed(.3)));
    
    driverController.a().onTrue(homeWrist);

    driverController.b().onTrue(new InstantCommand(() -> claw.setRotationSpeed(0)));

    driverController.rightBumper().onTrue(reverseArc);
    driverController.leftBumper().onTrue(arcing);
    // driverController.a().onTrue(rotate);

    driverController.x().onTrue(twoThirty);

    driverController.y().onTrue(ninetyRot);

    // driverController.rightBumper().onTrue(new InstantCommand(() -> claw.setArcingSpeed(-.1)));
    // driverController.leftBumper().onTrue(new InstantCommand(() -> claw.setArcingSpeed(.1)));
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
