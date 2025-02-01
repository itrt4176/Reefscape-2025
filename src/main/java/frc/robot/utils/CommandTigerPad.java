package frc.robot.utils;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;

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