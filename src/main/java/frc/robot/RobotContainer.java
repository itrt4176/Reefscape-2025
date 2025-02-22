// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.ShoulderJointConstants;
import frc.robot.Constants.ElbowJointConstants;
import java.io.File;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.events.EventTrigger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.ArmCommands;
import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.ArmJoint;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.ArmJoint.Position;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import swervelib.SwerveInputStream;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
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

  private final ArmCommands armCommands = new ArmCommands(shoulderJoint, elbowJoint);

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController = new CommandXboxController(
      OperatorConstants.kDriverControllerPort);

  private final SwerveSubsystem drivebase;

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

  // SwerveInputStream driveAngularVelocityKeyboard = SwerveInputStream
  //     .of(drivebase.getSwerveDrive(), () -> -m_driverController.getLeftY(),
  //         () -> -m_driverController.getLeftX())
  //     .withControllerRotationAxis(() -> m_driverController.getRawAxis(2))
  //     .deadband(OperatorConstants.DEADBAND).scaleTranslation(0.8)
  //     .allianceRelativeControl(true);
  // // Derive the heading axis with math!
  // SwerveInputStream driveDirectAngleKeyboard = driveAngularVelocityKeyboard
  //     .copy()
  //     .withControllerHeadingAxis(
  //         () -> Math.sin(m_driverController.getRawAxis(2) * Math.PI)
  //             * (Math.PI * 2),
  //         () -> Math.cos(m_driverController.getRawAxis(2) * Math.PI)
  //             * (Math.PI * 2))
  //     .headingWhile(true);



    private final SendableChooser<Command> autoChooser;
    private final SendableChooser<String> autoArmChooser;

    private final EventTrigger autoArmEvent;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    autoArmEvent = new EventTrigger("arm_position");

    // Disable the arm joints for tuning
    shoulderJoint.setEnabled(true);
    elbowJoint.setEnabled(true);

    // NamedCommands.registerCommand("Level One Place",
    //     armCommands.setPosition(Position.LEVEL_ONE));  

    // NamedCommands.registerCommand("Level Two Place",
    //     armCommands.setPosition(Position.LEVEL_TWO));  

    // NamedCommands.registerCommand("Level Three Place",
    //     armCommands.setPosition(Position.LEVEL_THREE));  

    // NamedCommands.registerCommand("Level Four Place",
    //     armCommands.setPosition(Position.LEVEL_FOUR)); 
        
    // NamedCommands.registerCommand("Intake Setpoint", 
    //     armCommands.setPosition(Position.INTAKE));

    // NamedCommands.registerCommand("Start/Climb", 
    //     armCommands.setPosition(Position.CLIMB).andThen(Commands.waitUntil(armCommands.atGoal())));

    drivebase = new SwerveSubsystem(
      new File(Filesystem.getDeployDirectory(), "swerve/neo"));

    // Configure the trigger bindings
    configureBindings();
    DriverStation.silenceJoystickConnectionWarning(true);

    //Apply inversion for inversion later
    Command joystickDrive = drivebase.driveCommand(
        () -> applyAllianceInversion(MathUtil.applyDeadband(m_driverController.getLeftY(), 0.1)), 
        () -> applyAllianceInversion(MathUtil.applyDeadband(m_driverController.getLeftX(), 0.1)), 
        () -> applyAllianceInversion(MathUtil.applyDeadband(m_driverController.getRightX(), 0.1)));


    drivebase.setDefaultCommand(joystickDrive);
      
    

    autoChooser = AutoBuilder.buildAutoChooser();

    SmartDashboard.putData("Auto", autoChooser);

    autoArmChooser = new SendableChooser<>();

    for (Position position : Position.values()) {
      autoArmChooser.addOption(position.toString(), position.name());
    }

    autoArmChooser.setDefaultOption(Position.LEVEL_FOUR.toString(), Position.LEVEL_FOUR.name());

    SmartDashboard.putData(autoArmChooser);
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


    m_driverController.a().onTrue(
      shoulderJoint.setPosition(Position.LEVEL_ONE)
        .alongWith(elbowJoint.setPosition(Position.LEVEL_ONE))
    );

    m_driverController.b().onTrue(
      shoulderJoint.setPosition(Position.INTAKE)
        .alongWith(elbowJoint.setPosition(Position.INTAKE))
    );

    m_driverController.x().onTrue(
      shoulderJoint.setPosition(Position.LEVEL_THREE)
        .alongWith(elbowJoint.setPosition(Position.LEVEL_THREE))
    );

    m_driverController.y().onTrue(
      shoulderJoint.setPosition(Position.LEVEL_FOUR)
        .alongWith(elbowJoint.setPosition(Position.LEVEL_FOUR))
    );
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous

    var position = Position.valueOf(autoArmChooser.getSelected());

    autoArmEvent.onTrue(armCommands.setPosition(position));
    
    // return autoChooser.getSelected();
    return drivebase.getAutonomousCommand("New Auto");
    // return armCommands.setPosition(Position.LEVEL_FOUR);
  }

  private static double applyAllianceInversion(double joystickInput) {
    var alliance = DriverStation.getAlliance();
    if (alliance.isPresent()) {
      return (alliance.get() == Alliance.Red) ? 1.0 * joystickInput : -1.0 * joystickInput;
    } else {
      return -1.0 * joystickInput;
    }
  }

  public void setMotorBrake(boolean brake) { drivebase.setMotorBrake(brake); }
}
