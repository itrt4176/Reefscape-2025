package frc.robot.utils;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.utils.TigerPad.LED;
import frc.robot.utils.TigerPad.LEDMode;

/**
 * A version of {@link TigerPad} with {@link Trigger} factories for command-based.
 * 
 * @see TigerPad
 */
public class CommandTigerPad extends CommandGenericHID {
    private final TigerPad tigerPad;

    /**
     * Construct an instance of a controller.
     * 
     * @param port The port index on the Driver Station that the conroller is plugged into.
     */
    public CommandTigerPad(int port) {
        super(port);
        tigerPad = new TigerPad(port);
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
     * @param led The LED to set the mode for.
     * @param mode The mode to set the LED to.
     * @return A command that sets the LED mode.
     */
    public Command setLED(LED led, LEDMode mode) {
        return new InstantCommand(() -> tigerPad.setLEDMode(led, mode), null);
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
     * Sets the LED mode for the level 2 left LED.
     * 
     * @param mode The mode to set the level 2 left LED to.
     * @return A command that sets the level 2 left LED mode.
     */
    public Command setLevel2LeftLED(LEDMode mode) {
        return setLED(TigerPad.LED.Level2Left, mode);
    }

    /**
     * Sets the LED mode for the level 2 right LED.
     * 
     * @param mode The mode to set the level 2 right LED to.
     * @return A command that sets the level 2 right LED mode.
     */
    public Command setLevel2RightLED(LEDMode mode) {
        return setLED(TigerPad.LED.Level2Right, mode);
    }

    /**
     * Sets the LED mode for the level 3 left LED.
     * 
     * @param mode The mode to set the level 3 left LED to.
     * @return A command that sets the level 3 left LED mode.
     */
    public Command setLevel3LeftLED(LEDMode mode) {
        return setLED(TigerPad.LED.Level3Left, mode);
    }

    /**
     * Sets the LED mode for the level 3 right LED.
     * 
     * @param mode The mode to set the level 3 right LED to.
     * @return A command that sets the level 3 right LED mode.
     */
    public Command setLevel3RightLED(LEDMode mode) {
        return setLED(TigerPad.LED.Level3Right, mode);
    }

    /**
     * Sets the LED mode for the level 4 left LED.
     * 
     * @param mode The mode to set the level 4 left LED to.
     * @return A command that sets the level 4 left LED mode.
     */
    public Command setLevel4LeftLED(LEDMode mode) {
        return setLED(TigerPad.LED.Level4Left, mode);
    }

    /**
     * Sets the LED mode for the level 4 right LED.
     * 
     * @param mode The mode to set the level 4 right LED to.
     * @return A command that sets the level 4 right LED mode.
     */
    public Command setLevel4RightLED(LEDMode mode) {
        return setLED(TigerPad.LED.Level4Right, mode);
    }

    /**
     * Constructs a Trigger instance around the intake button's digital signal.
     * 
     * @param loop the event loop instance to attach the event to.
     * @return a Trigger instance representing the intake button's digital signal
     *     attached to the given loop.
     */
    public Trigger intake(EventLoop loop) {
        return button(TigerPad.Button.Intake.value, loop);
    }

    /**
     * Constructs a Trigger instance around the intake button's digital signal.
     *
     * @return a Trigger instance representing the intake button's digital signal attached
     *     to the {@link CommandScheduler#getDefaultButtonLoop() default scheduler button loop}.
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
     *     attached to the given loop.
     */
    public Trigger level1(EventLoop loop) {
        return button(TigerPad.Button.Level1.value, loop);
    }

    /**
     * Constructs a Trigger instance around the level 1 button's digital signal.
     *
     * @return a Trigger instance representing the level 1 button's digital signal attached
     *     to the {@link CommandScheduler#getDefaultButtonLoop() default scheduler button loop}.
     * @see #level1(EventLoop)
     */
    public Trigger level1() {
        return level1(CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    /**
     * Constructs a Trigger instance around the level 2 left button's digital signal.
     * 
     * @param loop the event loop instance to attach the event to.
     * @return a Trigger instance representing the level 2 left button's digital signal
     *     attached to the given loop.
     */
    public Trigger level2Left(EventLoop loop) {
        return button(TigerPad.Button.Level2Left.value, loop);
    }

    /**
     * Constructs a Trigger instance around the level 2 left button's digital signal.
     *
     * @return a Trigger instance representing the level 2 left button's digital signal attached
     *     to the {@link CommandScheduler#getDefaultButtonLoop() default scheduler button loop}.
     * @see #level2Left(EventLoop)
     */
    public Trigger level2Left() {
        return level2Left(CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    /**
     * Constructs a Trigger instance around the level 2 right button's digital signal.
     * 
     * @param loop the event loop instance to attach the event to.
     * @return a Trigger instance representing the level 2 right button's digital signal
     *     attached to the given loop.
     */
    public Trigger level2Right(EventLoop loop) {
        return button(TigerPad.Button.Level2Right.value, loop);
    }

    /**
     * Constructs a Trigger instance around the level 2 right button's digital signal.
     *
     * @return a Trigger instance representing the level 2 right button's digital signal attached
     *     to the {@link CommandScheduler#getDefaultButtonLoop() default scheduler button loop}.
     * @see #level2Right(EventLoop)
     */
    public Trigger level2Right() {
        return level2Right(CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    /**
     * Constructs a Trigger instance around the level 3 left button's digital signal.
     * 
     * @param loop the event loop instance to attach the event to.
     * @return a Trigger instance representing the level 3 left button's digital signal
     *     attached to the given loop.
     */
    public Trigger level3Left(EventLoop loop) {
        return button(TigerPad.Button.Level3Left.value, loop);
    }

    /**
     * Constructs a Trigger instance around the level 3 left button's digital signal.
     *
     * @return a Trigger instance representing the level 3 left button's digital signal attached
     *     to the {@link CommandScheduler#getDefaultButtonLoop() default scheduler button loop}.
     * @see #level3Left(EventLoop)
     */
    public Trigger level3Left() {
        return level3Left(CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    /**
     * Constructs a Trigger instance around the level 3 right button's digital signal.
     * 
     * @param loop the event loop instance to attach the event to.
     * @return a Trigger instance representing the level 3 right button's digital signal
     *     attached to the given loop.
     */
    public Trigger level3Right(EventLoop loop) {
        return button(TigerPad.Button.Level3Right.value, loop);
    }

    /**
     * Constructs a Trigger instance around the level 3 right button's digital signal.
     *
     * @return a Trigger instance representing the level 3 right button's digital signal attached
     *     to the {@link CommandScheduler#getDefaultButtonLoop() default scheduler button loop}.
     * @see #level3Right(EventLoop)
     */
    public Trigger level3Right() {
        return level3Right(CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    /**
     * Constructs a Trigger instance around the level 4 left button's digital signal.
     * 
     * @param loop the event loop instance to attach the event to.
     * @return a Trigger instance representing the level 4 left button's digital signal
     *     attached to the given loop.
     */
    public Trigger level4Left(EventLoop loop) {
        return button(TigerPad.Button.Level4Left.value, loop);
    }

    /**
     * Constructs a Trigger instance around the level 4 left button's digital signal.
     *
     * @return a Trigger instance representing the level 4 left button's digital signal attached
     *     to the {@link CommandScheduler#getDefaultButtonLoop() default scheduler button loop}.
     * @see #level4Left(EventLoop)
     */
    public Trigger level4Left() {
        return level4Left(CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    /**
     * Constructs a Trigger instance around the level 4 right button's digital signal.
     * 
     * @param loop the event loop instance to attach the event to.
     * @return a Trigger instance representing the level 4 right button's digital signal
     *     attached to the given loop.
     */
    public Trigger level4Right(EventLoop loop) {
        return button(TigerPad.Button.Level4Right.value, loop);
    }

    /**
     * Constructs a Trigger instance around the level 4 right button's digital signal.
     *
     * @return a Trigger instance representing the level 4 right button's digital signal attached
     *     to the {@link CommandScheduler#getDefaultButtonLoop() default scheduler button loop}.
     * @see #level4Right(EventLoop)
     */
    public Trigger level4Right() {
        return level4Right(CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    /**
     * Constructs a Trigger instance around the arm override switch's digital signal.
     * 
     * @param loop the event loop instance to attach the event to.
     * @return a Trigger instance representing the arm override switch's digital signal
     *     attached to the given loop.
     */
    public Trigger armOverride(EventLoop loop) {
        return button(TigerPad.ToggleSwitch.ArmOverride.value, loop);
    }

    /**
     * Constructs a Trigger instance around the arm override switch's digital signal.
     *
     * @return a Trigger instance representing the arm override switch's digital signal attached
     *     to the {@link CommandScheduler#getDefaultButtonLoop() default scheduler button loop}.
     * @see #armOverride(EventLoop)
     */
    public Trigger armOverride() {
        return armOverride(CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    /**
     * Constructs a Trigger instance around the wrist override switch's digital signal.
     * 
     * @param loop the event loop instance to attach the event to.
     * @return a Trigger instance representing the wrist override switch's digital signal
     *     attached to the given loop.
     */
    public Trigger wristOverride(EventLoop loop) {
        return button(TigerPad.ToggleSwitch.WristOverride.value, loop);
    }

    /**
     * Constructs a Trigger instance around the wrist override switch's digital signal.
     *
     * @return a Trigger instance representing the wrist override switch's digital signal attached
     *     to the {@link CommandScheduler#getDefaultButtonLoop() default scheduler button loop}.
     * @see #wristOverride(EventLoop)
     */
    public Trigger wristOverride() {
        return wristOverride(CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    /**
     * Get the lower arm axis value of the controller.
     *
     * @return The axis value.
     */
    public double getLowerArm() {
        return tigerPad.getLowerArm();
    }

    /**
     * Get the upper arm axis value of the controller.
     *
     * @return The axis value.
     */
    public double getUpperArm() {
        return tigerPad.getUpperArm();
    }

    /**
     * Get the wrist pivot axis value of the controller.
     *
     * @return The axis value.
     */
    public double getWristPivot() {
        return tigerPad.getWristPivot();
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