package frc.robot.utils;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.utils.TigerPad.LED;
import frc.robot.utils.TigerPad.LEDMode;

/**
 * A version of {@link TigerPad} with {@link Trigger} factories for
 * command-based.
 * 
 * @see TigerPad
 */
public class CommandTigerPad extends CommandGenericHID {
  private static CommandTigerPad instance;

  /**
   * Returns the singleton instance of the CommandTigerPad class for the
   * specified port. If the instance does not exist, it will be created.
   * Subsequent calls to this method will return the same instance
   * regardless of the value of port.
   *
   * @param port the port index on the Driver Station that the conroller is
   *             plugged into. (0-5)
   * @return the singleton instance of the CommandTigerPad class
   */
  public static CommandTigerPad getInstance(final int port) {
    if (instance == null) {
      instance = new CommandTigerPad(port);
    }

    return instance;
  }

  private final TigerPad tigerPad;

  /**
   * Construct an instance of a controller.
   * 
   * @param port The port index on the Driver Station that the conroller is
   *             plugged into.
   */
  private CommandTigerPad(final int port) {
    super(port);
    tigerPad = TigerPad.getInstance(port);
  }

  /**
   * Get the underlying TigerPad object.
   * 
   * @return the wrapped TigerPad object
   */
  @Override
  public TigerPad getHID() {
    return tigerPad;
  }

  /**
   * Sets the LED mode for the specified LED.
   * 
   * @param led  The LED to set the mode for.
   * @param mode The mode to set the LED to.
   * @return A command that sets the LED mode.
   */
  public Command setLED(LED led, LEDMode mode) {
    return Commands.runOnce(() -> tigerPad.setLEDMode(led, mode)).ignoringDisable(true);
  }

  /**
   * Sets the LED mode for all LEDs.
   * 
   * @param mode The mode to set the LED to.
   * @return A command that sets the LED mode.
   */
  public Command setAllLEDs(LEDMode mode) {
    Command command = new InstantCommand();

    for (LED led : LED.values()) {
      command = command.andThen(setLED(led, mode));
    }

    return command.ignoringDisable(true);
  }

  /**
   * Sets the LED mode for the intake LED.
   * 
   * @param mode The mode to set the intake LED to.
   * @return A command that sets the intake LED mode.
   */
  public Command setIntakeLED(LEDMode mode) {
    return setLED(TigerPad.LED.Intake, mode);
  }

  /**
   * Sets the LED mode for the level 1 LED.
   * 
   * @param mode The mode to set the level 1 LED to.
   * @return A command that sets the level 1 LED mode.
   */
  public Command setLevel1LED(LEDMode mode) {
    return setLED(TigerPad.LED.Level1, mode);
  }

  /**
   * Sets the LED mode for the level 2 LED.
   * 
   * @param mode The mode to set the level 2 LED to.
   * @return A command that sets the level 2 LED mode.
   */
  public Command setLevel2LED(LEDMode mode) {
    return setLED(TigerPad.LED.Level2, mode);
  }

  /**
   * Sets the LED mode for the arm flat LED.
   * 
   * @param mode The mode to set the arm flat LED to.
   * @return A command that sets the arm flat LED mode.
   */
  public Command setArmFlatLED(LEDMode mode) {
    return setLED(TigerPad.LED.FlatArm, mode);
  }

  /**
   * Sets the LED mode for the level 3 LED.
   * 
   * @param mode The mode to set the level 3 LED to.
   * @return A command that sets the level 3 LED mode.
   */
  public Command setLevel3LED(LEDMode mode) {
    return setLED(TigerPad.LED.Level3, mode);
  }

  /**
   * Sets the LED mode for the low algae LED.
   * 
   * @param mode The mode to set the low algae LED to.
   * @return A command that sets the low algae LED mode.
   */
  public Command setLowAlgaeLED(LEDMode mode) {
    return setLED(TigerPad.LED.LowAlgae, mode);
  }

  /**
   * Sets the LED mode for the level 4 LED.
   * 
   * @param mode The mode to set the level 4 LED to.
   * @return A command that sets the level 4 LED mode.
   */
  public Command setLevel4LED(LEDMode mode) {
    return setLED(TigerPad.LED.Level4, mode);
  }

  /**
   * Sets the LED mode for the high algae LED.
   * 
   * @param mode The mode to set the high algae LED to.
   * @return A command that sets the high algae LED mode.
   */
  public Command setHighAlgaeLED(LEDMode mode) {
    return setLED(TigerPad.LED.HighAlgae, mode);
  }

  /**
   * Constructs a Trigger instance around the intake button's digital signal.
   * 
   * @param loop the event loop instance to attach the event to.
   * @return a Trigger instance representing the intake button's digital signal
   *         attached to the given loop.
   */
  public Trigger intake(EventLoop loop) {
    return button(TigerPad.Button.Intake.value, loop);
  }

  /**
   * Constructs a Trigger instance around the intake button's digital signal.
   *
   * @return a Trigger instance representing the intake button's digital signal
   *         attached
   *         to the {@link CommandScheduler#getDefaultButtonLoop() default
   *         scheduler button loop}.
   * @see #intake(EventLoop)
   */
  public Trigger intake() {
    return intake(CommandScheduler.getInstance().getDefaultButtonLoop());
  }

  /**
   * Constructs a Trigger instance around the level 1 button's digital signal.
   * 
   * @param loop the event loop instance to attach the event to.
   * @return a Trigger instance representing the level 1 button's digital signal
   *         attached to the given loop.
   */
  public Trigger level1(EventLoop loop) {
    return button(TigerPad.Button.Level1.value, loop);
  }

  /**
   * Constructs a Trigger instance around the level 1 button's digital signal.
   *
   * @return a Trigger instance representing the level 1 button's digital signal
   *         attached
   *         to the {@link CommandScheduler#getDefaultButtonLoop() default
   *         scheduler button loop}.
   * @see #level1(EventLoop)
   */
  public Trigger level1() {
    return level1(CommandScheduler.getInstance().getDefaultButtonLoop());
  }

  /**
   * Constructs a Trigger instance around the level 2 button's digital
   * signal.
   * 
   * @param loop the event loop instance to attach the event to.
   * @return a Trigger instance representing the level 2 button's digital
   *         signal
   *         attached to the given loop.
   */
  public Trigger level2(EventLoop loop) {
    return button(TigerPad.Button.Level2.value, loop);
  }

  /**
   * Constructs a Trigger instance around the level 2 button's digital
   * signal.
   *
   * @return a Trigger instance representing the level 2 button's digital
   *         signal attached
   *         to the {@link CommandScheduler#getDefaultButtonLoop() default
   *         scheduler button loop}.
   * @see #level2(EventLoop)
   */
  public Trigger level2() {
    return level2(CommandScheduler.getInstance().getDefaultButtonLoop());
  }

  /**
   * Constructs a Trigger instance around the arm flat button's digital
   * signal.
   * 
   * @param loop the event loop instance to attach the event to.
   * @return a Trigger instance representing the arm flat button's digital
   *         signal
   *         attached to the given loop.
   */
  public Trigger armFlat(EventLoop loop) {
    return button(TigerPad.Button.ArmFlat.value, loop);
  }

  /**
   * Constructs a Trigger instance around the arm flat button's digital
   * signal.
   *
   * @return a Trigger instance representing the arm flat button's digital
   *         signal attached
   *         to the {@link CommandScheduler#getDefaultButtonLoop() default
   *         scheduler button loop}.
   * @see #armFlat(EventLoop)
   */
  public Trigger armFlat() {
    return armFlat(CommandScheduler.getInstance().getDefaultButtonLoop());
  }

  /**
   * Constructs a Trigger instance around the level 3 button's digital
   * signal.
   * 
   * @param loop the event loop instance to attach the event to.
   * @return a Trigger instance representing the level 3 button's digital
   *         signal
   *         attached to the given loop.
   */
  public Trigger level3(EventLoop loop) {
    return button(TigerPad.Button.Level3.value, loop);
  }

  /**
   * Constructs a Trigger instance around the level 3 button's digital
   * signal.
   *
   * @return a Trigger instance representing the level 3 button's digital
   *         signal attached
   *         to the {@link CommandScheduler#getDefaultButtonLoop() default
   *         scheduler button loop}.
   * @see #level3(EventLoop)
   */
  public Trigger level3() {
    return level3(CommandScheduler.getInstance().getDefaultButtonLoop());
  }

  /**
   * Constructs a Trigger instance around the low algae button's digital
   * signal.
   * 
   * @param loop the event loop instance to attach the event to.
   * @return a Trigger instance representing the low algae button's digital
   *         signal
   *         attached to the given loop.
   */
  public Trigger lowAlgae(EventLoop loop) {
    return button(TigerPad.Button.LowAlgae.value, loop);
  }

  /**
   * Constructs a Trigger instance around the low algae button's digital
   * signal.
   *
   * @return a Trigger instance representing the low algae button's digital
   *         signal attached
   *         to the {@link CommandScheduler#getDefaultButtonLoop() default
   *         scheduler button loop}.
   * @see #lowAlgae(EventLoop)
   */
  public Trigger lowAlgae() {
    return lowAlgae(CommandScheduler.getInstance().getDefaultButtonLoop());
  }

  /**
   * Constructs a Trigger instance around the level 4 button's digital
   * signal.
   * 
   * @param loop the event loop instance to attach the event to.
   * @return a Trigger instance representing the level 4 button's digital
   *         signal
   *         attached to the given loop.
   */
  public Trigger level4(EventLoop loop) {
    return button(TigerPad.Button.Level4.value, loop);
  }

  /**
   * Constructs a Trigger instance around the level 4 button's digital
   * signal.
   *
   * @return a Trigger instance representing the level 4 button's digital
   *         signal attached
   *         to the {@link CommandScheduler#getDefaultButtonLoop() default
   *         scheduler button loop}.
   * @see #level4(EventLoop)
   */
  public Trigger level4() {
    return level4(CommandScheduler.getInstance().getDefaultButtonLoop());
  }

  /**
   * Constructs a Trigger instance around the high algae button's digital
   * signal.
   * 
   * @param loop the event loop instance to attach the event to.
   * @return a Trigger instance representing the high algae button's digital
   *         signal
   *         attached to the given loop.
   */
  public Trigger highAlgae(EventLoop loop) {
    return button(TigerPad.Button.HighAlgae.value, loop);
  }

  /**
   * Constructs a Trigger instance around the high algae button's digital
   * signal.
   *
   * @return a Trigger instance representing the high algae button's digital
   *         signal attached
   *         to the {@link CommandScheduler#getDefaultButtonLoop() default
   *         scheduler button loop}.
   * @see #highAlgae(EventLoop)
   */
  public Trigger highAlgae() {
    return highAlgae(CommandScheduler.getInstance().getDefaultButtonLoop());
  }

  /**
   * Constructs a Trigger instance around the arm override switch's digital
   * signal.
   * 
   * @param loop the event loop instance to attach the event to.
   * @return a Trigger instance representing the arm override switch's digital
   *         signal
   *         attached to the given loop.
   */
  public Trigger armOverride(EventLoop loop) {
    return button(TigerPad.ToggleSwitch.ArmOverride.value, loop);
  }

  /**
   * Constructs a Trigger instance around the arm override switch's digital
   * signal.
   *
   * @return a Trigger instance representing the arm override switch's digital
   *         signal attached
   *         to the {@link CommandScheduler#getDefaultButtonLoop() default
   *         scheduler button loop}.
   * @see #armOverride(EventLoop)
   */
  public Trigger armOverride() {
    return armOverride(CommandScheduler.getInstance().getDefaultButtonLoop());
  }

  /**
   * Constructs a Trigger instance around the wrist override switch's digital
   * signal.
   * 
   * @param loop the event loop instance to attach the event to.
   * @return a Trigger instance representing the wrist override switch's digital
   *         signal
   *         attached to the given loop.
   */
  public Trigger wristOverride(EventLoop loop) {
    return button(TigerPad.ToggleSwitch.WristOverride.value, loop);
  }

  /**
   * Constructs a Trigger instance around the wrist override switch's digital
   * signal.
   *
   * @return a Trigger instance representing the wrist override switch's digital
   *         signal attached
   *         to the {@link CommandScheduler#getDefaultButtonLoop() default
   *         scheduler button loop}.
   * @see #wristOverride(EventLoop)
   */
  public Trigger wristOverride() {
    return wristOverride(CommandScheduler.getInstance().getDefaultButtonLoop());
  }

  /**
   * Get the shoulder joint axis value of the controller.
   *
   * @return The axis value.
   */
  public double getShoulderJoint() {
    return tigerPad.getShoulderJoint();
  }

  /**
   * Get the elbow joint axis value of the controller.
   *
   * @return The axis value.
   */
  public double getElbowJoint() {
    return tigerPad.getElbowJoint();
  }

  /**
   * Get the wrist arc axis value of the controller.
   *
   * @return The axis value.
   */
  public double getWristArc() {
    return tigerPad.getWristArc();
  }

  /**
   * @see TigerPad#zeroWristDial()
   *
   * @return A command that runs zeroes the wrist dial.
   */
  public Command zeroWristDial() {
    return Commands.runOnce(tigerPad::zeroWristDial);
  }

  /**
   * Gets the wrist rotation in degrees.
   * 
   * This method retrieves the raw axis value for wrist rotation, adjusts it by
   * subtracting the dial offset, and then maps the adjusted value from the range
   * [-0.5, 0.5] to the range [-180.0, 180.0] degrees.
   * 
   * <p>
   * To get the unitless wrist dial value without offset, use {@link #getHID()}
   * and
   * {@link TigerPad#getRawAxis(int)}.
   * 
   * @return The wrist rotation in degrees, ranging from -180.0 to 180.0.
   */
  public double getWristDegrees() {
    return tigerPad.getWristDegrees();
  }

  /**
   * Gets the wrist rotation in radians.
   *
   * This method retrieves the raw axis value for wrist rotation, adjusts it by
   * subtracting
   * the dial offset, and then maps the adjusted value from the range [-0.5, 0.5]
   * to the range
   * [-π, π].
   * 
   * <p>
   * To get the unitless wrist dial value without offset, use {@link #getHID()}
   * and
   * {@link TigerPad#getRawAxis(int)}.
   *
   * @return The wrist rotation in radians -π to π.
   */
  public double getWristRadians() {
    return tigerPad.getWristRadians();
  }

  @Override
  public Trigger pov(int pov, int angle, EventLoop loop) {
    throw new UnsupportedOperationException("POV methods are not supported by TigerPad.");
  }

  @Override
  public Trigger pov(int angle) {
    throw new UnsupportedOperationException("POV methods are not supported by TigerPad.");
  }

  @Override
  public Trigger povUpRight() {
    throw new UnsupportedOperationException("POV methods are not supported by TigerPad.");
  }

  @Override
  public Trigger povUp() {
    throw new UnsupportedOperationException("POV methods are not supported by TigerPad.");
  }

  @Override
  public Trigger povUpLeft() {
    throw new UnsupportedOperationException("POV methods are not supported by TigerPad.");
  }

  @Override
  public Trigger povDown() {
    throw new UnsupportedOperationException("POV methods are not supported by TigerPad.");
  }

  @Override
  public Trigger povLeft() {
    throw new UnsupportedOperationException("POV methods are not supported by TigerPad.");
  }

  @Override
  public Trigger povRight() {
    throw new UnsupportedOperationException("POV methods are not supported by TigerPad.");
  }

  @Override
  public Trigger povDownLeft() {
    throw new UnsupportedOperationException("POV methods are not supported by TigerPad.");
  }

  @Override
  public Trigger povDownRight() {
    throw new UnsupportedOperationException("POV methods are not supported by TigerPad.");
  }

  @Override
  public Trigger povCenter() {
    throw new UnsupportedOperationException("POV methods are not supported by TigerPad.");
  }

  @Override
  public void setRumble(RumbleType type, double value) {
    throw new UnsupportedOperationException("Rumble is not supported by TigerPad.");
  }
}