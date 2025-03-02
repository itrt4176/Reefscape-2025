// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.ShoulderJointConstants;
import frc.robot.Constants.ClawConstants;
import frc.robot.Constants.ElbowJointConstants;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.wpilibj2.command.Commands.*;

import java.io.File;
import java.util.Set;
import java.util.function.Function;
import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.IntakePositioning;
import frc.robot.subsystems.Intake;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
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
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.ArmJoint;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.ArmJoint.Position;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import swervelib.SwerveInputStream;
import frc.robot.utils.BrakingMotors;
import frc.robot.utils.CommandTigerPad;
import frc.robot.utils.TigerPad.LEDMode;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
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
    OperatorConstants.DRIVE_CONTROLLER_PORT
  );

  private final CommandTigerPad armControlPanel = CommandTigerPad.getInstance(
    OperatorConstants.ARM_CONTROL_PANEL_PORT
  );

  private final ArmJoint shoulderJoint = new ArmJoint(
    ShoulderJointConstants.motorPort,
    ShoulderJointConstants.encoderPort,
    ShoulderJointConstants.encoderOffset,
    ShoulderJointConstants.pidConfig,
    ShoulderJointConstants.angleMap,
    "Shoulder Joint",
    false,
    0.0,
    180.0
  );

  private final ArmJoint elbowJoint = new ArmJoint(
    ElbowJointConstants.motorPort,
    ElbowJointConstants.encoderPort,
    ElbowJointConstants.encoderOffset,
    () -> shoulderJoint.getAngle().magnitude(),
    ElbowJointConstants.pidConfig,
    ElbowJointConstants.angleMap,
    "Elbow Joint",
    false,
    -35.0,
    180.0
  );

  private final ArmCommands armCommands = new ArmCommands(shoulderJoint, elbowJoint);

  private Climber climber = new Climber();

  private final SwerveSubsystem drivebase = new SwerveSubsystem(
    new File(Filesystem.getDeployDirectory(), "swerve/neo")
  );

  private final Intake intake = new Intake();
  private final IntakePositioning storeIntake = new IntakePositioning(intake, IntakeConstants.STORE_ANGLE);
  private final IntakePositioning intakeDown = new IntakePositioning(intake, IntakeConstants.INTAKE_DOWN);

  private final BrakingMotors[] brakingSubsystems = {drivebase, shoulderJoint, elbowJoint, claw};
  private Trigger robotEnabled = new Trigger(RobotState::isEnabled);

  private final SendableChooser<Command> autoChooser = new SendableChooser<>(); 

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Disable the arm joints for tuning
    // shoulderJoint.setEnabled(false);
    // elbowJoint.setEnabled(false);

    // Configure the trigger bindings
    configureBindings();
    // configureSysIdBindings();
    DriverStation.silenceJoystickConnectionWarning(true);

    //Apply inversion for inversion later
    Command joystickDrive = drivebase.driveCommand(
                    () -> MathUtil.applyDeadband(-driverController.getLeftY(), 0.1), 
                    () -> MathUtil.applyDeadband(-driverController.getLeftX(), 0.1), 
                    () -> MathUtil.applyDeadband(-driverController.getRightX(), 0.1));


    drivebase.setDefaultCommand(joystickDrive);

    autoChooser.setDefaultOption("Do Nothing", none());
    autoChooser.addOption("Leave starting position", drivebase.driveToDistanceCommand(Units.inchesToMeters(60), -0.25));
    autoChooser.addOption(
      "Place L1", 
      new HomeWrist(claw).andThen(
        waitSeconds(0.75),
        setWristAndArm(
          ClawConstants.L1_ARC,
          ClawConstants.L1_ROT,
          Position.LEVEL_ONE,
          armControlPanel::setLevel1LED
        ),
        drivebase.driveToDistanceCommand(Units.inchesToMeters(70 - 10), -0.25)
      )
    );
    
    SmartDashboard.putData(autoChooser);
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
    robotEnabled.onChange(defer(() -> {
      var command = none();

      for (BrakingMotors subsystem : brakingSubsystems) {
        command = command.alongWith(subsystem.enableMotorBrakes(RobotState.isEnabled()));
      }

      return command;
    }, Set.of()));

    driverController.a().whileTrue(startEnd(() -> claw.setGripSpeed(-0.30), () -> claw.setGripSpeed(0), claw));

    driverController.b().whileTrue(startEnd(() -> claw.setGripSpeed(0.30), () -> claw.setGripSpeed(0), claw));

    driverController.x().toggleOnTrue(
      startEnd(
        () -> {
          intake.setPivotBrake(false);
          intake.setPivotAngle(IntakeConstants.INTAKE_DOWN);
          intake.setSpeed(1.0);
        },
        () -> {
          intake.setPivotBrake(true);
          intake.setPivotAngle(IntakeConstants.STORE_ANGLE);
          intake.setSpeed(0.0);
        }
      )
    );

    driverController.y().toggleOnTrue(new StartEndCommand(
      () -> intake.setSpeed(-1.0), 
      () -> intake.setSpeed(0) 
    ));

    driverController.leftBumper().whileTrue(new StartEndCommand(() -> climber.setWinchSpeed(1.0), () -> climber.setWinchSpeed(0), climber));
    driverController.rightBumper().whileTrue(new StartEndCommand(() -> climber.setWinchSpeed(-1.0), () -> climber.setWinchSpeed(0), climber));


    driverController.leftTrigger(0.5).whileTrue(
      startEnd(
        () -> drivebase.enableSlowMode(true),
        () -> drivebase.enableSlowMode(false)
      )
    );

    armControlPanel.intake().or(driverController.povLeft()).onTrue(
      armControlPanel.setAllLEDs(LEDMode.Off).andThen(
        armControlPanel.setIntakeLED(LEDMode.Blink),
        parallel(
          elbowJoint.setPosition(Position.INTAKE).andThen(
            waitUntil(() -> elbowJoint.getAngle().in(Degrees) >= 50),
            shoulderJoint.setPosition(Position.INTAKE)
          ),
          setWrist(ClawConstants.INTAKE_ARC, ClawConstants.INTAKE_ROT)
        ),
        armControlPanel.setIntakeLED(LEDMode.On)
      )  
    );

    armControlPanel.level1().or(driverController.povDown()).onTrue(
      setWristAndArm(
        ClawConstants.L1_ARC,
        ClawConstants.L1_ROT,
        Position.LEVEL_ONE,
        armControlPanel::setLevel1LED
      )
    );

    // For next comp
    // armControlPanel.level2().onTrue(
    //   setWristAndArm(
    //     ClawConstants.L2_ARC,
    //     ClawConstants.L2_ROT,
    //     Position.LEVEL_TWO,
    //     armControlPanel::setLevel2LED
    //   )
    // );

    armControlPanel.armFlat().onTrue(
      setWristAndArm(
        ClawConstants.CLIMB_ARC,
        ClawConstants.CLIMB_ROT,
        Position.CLIMB,
        armControlPanel::setArmFlatLED
      )
    );

    armControlPanel.level3().or(driverController.povRight()).onTrue(
      armControlPanel.setAllLEDs(LEDMode.Off).andThen(
        armControlPanel.setLevel3LED(LEDMode.Blink),
        parallel(
          elbowJoint.setPosition(Position.LEVEL_THREE).andThen(
            waitUntil(() -> elbowJoint.getAngle().in(Degrees) <= 135),
            shoulderJoint.setPosition(Position.LEVEL_THREE)
          ),
          setWrist(ClawConstants.L3_ARC, ClawConstants.L3_ROT)
        ),
        armControlPanel.setLevel3LED(LEDMode.On)
      )  
    );

    armControlPanel.lowAlgae().onTrue(
      setWristAndArm(
        ClawConstants.LOW_ALGAE_ARC,
        ClawConstants.LOW_ALGAE_ROT,
        Position.LOW_ALGAE,
        armControlPanel::setLowAlgaeLED
      )
    );

    armControlPanel.level4().or(driverController.povUp()).onTrue(
      setWristAndArm(
        ClawConstants.L4_ARC,
        ClawConstants.L4_ROT,
        Position.LEVEL_FOUR,
        armControlPanel::setLevel4LED
      )
    );

    armControlPanel.highAlgae().onTrue(
      setWristAndArm(
        ClawConstants.HIGH_ALGAE_ARC,
        ClawConstants.HIGH_ALGAE_ROT,
        Position.HIGH_ALGAE,
        armControlPanel::setHighAlgaeLED
      )
    );

    
    armControlPanel.armOverride().whileTrue(armCommands.adjustOffset(armControlPanel::getShoulderJoint, armControlPanel::getElbowJoint));

    driverController.start().onTrue(homeWrist);

    claw.homed().onTrue(runOnce(claw::zeroRotation));
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

  private Command setWrist(double arcAngle, double rotationAngle) {
    return new RotationSetpoint(claw, rotationAngle).andThen(new ClawSetArcAngle(claw, arcAngle));
  }

 

  private Command setWristAndArm(
    double arcAngle,
    double rotationAngle,
    Position armPosition,
    Function<LEDMode, Command> setLEDCommand
  ) {
    return armControlPanel.setAllLEDs(LEDMode.Off).andThen(
      setLEDCommand.apply(LEDMode.Blink),
      parallel(
        armCommands.setPosition(armPosition),
        setWrist(arcAngle, rotationAngle)
      ),
      setLEDCommand.apply(LEDMode.On)
    );
  }

    

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    // return armCommands.setPosition(Position.LEVEL_ONE).andThen(
    //   waitUntil(armCommands.atGoal()).withTimeout(1.5),
    //   new HomeWrist(claw),
    //   setWrist(ClawConstants.L1_ARC, ClawConstants.L1_ROT),
    //   drivebase.driveToDistanceCommand(Units.inchesToMeters(70 - 10), -0.25)
    // );

    // return new HomeWrist(claw).andThen(
    //   waitSeconds(.075),
    //   setWristAndArm(
    //     ClawConstants.L1_ARC,
    //     ClawConstants.L1_ROT,
    //     Position.LEVEL_ONE,
    //     armControlPanel::setLevel1LED
    //   ),
    //   drivebase.driveToDistanceCommand(Units.inchesToMeters(70 - 10), -0.25)
    // );

    // return drivebase.driveToDistanceCommand(Units.inchesToMeters(60), -0.25);

    return autoChooser.getSelected();
  }

  public void setMotorBrake(boolean brake) { drivebase.setMotorBrake(brake); }
}