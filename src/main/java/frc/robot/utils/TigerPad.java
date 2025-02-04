package frc.robot.utils;

import java.util.HashMap;
import java.util.Locale;
import java.util.Map;
import java.util.NoSuchElementException;

import org.apache.commons.lang3.StringUtils;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.networktables.IntegerEntry;
import edu.wpi.first.networktables.IntegerTopic;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.event.BooleanEvent;
import edu.wpi.first.wpilibj.event.EventLoop;

/**
 * Handle input from TigerPad controllers connected to the Driver Station.
 *
 * <p>This class handles TigerPad input that comes from the Driver Station. Each time a value is
 * requested the most recent value is returned. There is a single class instance for each controller
 * and the mapping of ports to hardware buttons depends on the code in the Driver Station.
 *
 * <p>Only controllers connected through the official NI DS are guaranteed to have the correct mapping, and
 * Sim is not guaranteed to have the same mapping.
 */
public class TigerPad extends GenericHID implements Sendable {
    private static TigerPad instance;

    /**
     * Returns the singleton instance of the TigerPad class for the specified port.
     * If the instance does not exist, it will be created. Subsequent calls to this
     * method will return the same instance regardless of the value of port.
     *
     * @param port the port index on the Driver Station that the conroller is plugged into. (0-5)
     * @return the singleton instance of the TigerPad class
     */
    public static TigerPad getInstance(final int port) {
        if (instance == null) {
            instance = new TigerPad(port);
        }

        return instance;
    }
    
    private static final NetworkTable ledTable = NetworkTableInstance
        .getDefault()
        .getTable("tigerpad")
        .getSubTable("leds");
    private double dialOffset;

    /** Represents a digital button on a TigerPad. */
    public enum Button {
        /** Intake button. */
        Intake(1),
        /** Level 1 button. */
        Level1(2),
        /** Level 2 left button. */
        Level2Left(3),
        /** Level 2 right button. */
        Level2Right(4),
        /** Level 3 left button. */
        Level3Left(5),
        /** Level 3 right button. */
        Level3Right(6),
        /** Level 4 left button. */
        Level4Left(7),
        /** Level 4 right button. */
        Level4Right(8);

        /** Button value. */
        public final int value;

        Button(int value) {
            this.value = value;
        }

        /** Get the human-friendly name of the button, matching the relevant methods. This is done by
         * appending `Button`.
         * 
         * <p>Primarily used for automated unit tests.
         * 
         * @return the human-friendly name of the button.
        */
        @Override
        public String toString() {
            return this.name() + "Button";
        }
    }

    /** Represents a toggle switch on a TigerPad. */
    public enum ToggleSwitch {
        /** Arm override switch. */
        ArmOverride(9),
        /** Wrist override switch. */
        WristOverride(10);

        /** Switch value. */
        public final int value;

        ToggleSwitch(int value) {
            this.value = value;
        }

        /** Get the human-friendly name of the switch, matching the relevant methods. This is done by
         * appending `Switch`.
         * 
         * <p>Primarily used for automated unit tests.
         * 
         * @return the human-friendly name of the switch.
        */
        @Override
        public String toString() {
            return this.name() + "Switch";
        }
    }

    /** Represents an axis on a TigerPad. */
    public enum Axis {
        /** Lower arm axis. */
        LowerArm(0),
        /** Upper arm axis. */
        UpperArm(1),
        /** Wrist pivot axis. */
        WristPivot(2),
        /** Wrist rotation axis. */
        WristRotation(3);

        /** Axis value. */
        public final int value;

        Axis(int value) {
            this.value = value;
        }

        /** Get the human-friendly name of the axis, matching the relevant methods. This is done by
         * appending `Axis`.
         * 
         * <p>Primarily used for automated unit tests.
         * 
         * @return the human-friendly name of the axis.
        */
        @Override
        public String toString() {
            return this.name() + "Axis";
        }
    }

    /** Represents an LED on a TigerPad. */
    public enum LED {
        /** Intake LED. */
        Intake(0),
        /** Level 1 LED. */
        Level1(1),
        /** Level 2 left LED. */
        Level2Left(2),
        /** Level 2 right LED. */
        Level2Right(3),
        /** Level 3 left LED. */
        Level3Left(4),
        /** Level 3 right LED. */
        Level3Right(5),
        /** Level 4 left LED. */
        Level4Left(6),
        /** Level 4 right LED. */
        Level4Right(7);

        /** LED value. */
        public final int value;
        private final NetworkTable table;
        private final IntegerTopic topic;
        private final IntegerEntry entry;

        LED(int value) {
            this.value = value;
            table = TigerPad.ledTable.getSubTable(Integer.toString(value));
            topic = table.getIntegerTopic("value");
            entry = topic.getEntry(LEDMode.Off.value);
            entry.set(LEDMode.Off.value);

            table.getStringTopic("name").publish().set(this.toString());
        }

        /** Get the human-friendly name of the LED, matching the relevant methods. This is done by
         * appending `LED`.
         * 
         * <p>Primarily used for automated unit tests.
         * 
         * @return the human-friendly name of the LED.
        */
        @Override
        public String toString() {
            return this.name() + "LED";
        }
    }

    /** Represents an LED mode on a TigerPad. */
    public enum LEDMode {
        /** LED off mode. */
        Off(0),
        /** LED blink mode. */
        Blink(1),
        /** LED on mode. */
        On(2);

        private static final Map<Integer, LEDMode> ledsByValue = new HashMap<>();

        static {
            for (LEDMode mode : LEDMode.values()) {
                ledsByValue.put(mode.value, mode);
            }
        }

        private static LEDMode getByValue(int value) {
            if (value > ledsByValue.size() - 1) {
                throw new NoSuchElementException(value + " is not a valid LEDMode.");
            }

            return ledsByValue.get(value);
        }

        private static LEDMode safeValueOf(String name) {
            name = name.toLowerCase(Locale.ROOT).strip();
            name = StringUtils.removeEnd(name, "ing");
            name = StringUtils.capitalize(name);

            try {
                return LEDMode.valueOf(name);
            } catch (IllegalArgumentException e) {
                return null;
            }
        }

        /** LED mode value. */
        public final int value;

        LEDMode(int value) {
            this.value = value;
        }

        /** Get the human-friendly name of the LED mode, matching the relevant methods. This is done by
         * appending `ing` to `Blink` and converting the name to lowercase.
         * 
         * <p>Primarily used for automated unit tests.
         * 
         * @return the human-friendly name of the LED mode.
        */
        @Override
        public String toString() {
            if (this == Blink) {
                return this.name().toLowerCase() + "ing";
            }

            return this.name().toLowerCase();
        }
    }

    /**
     * Construct an instance of a controller.
     *
     * @param port The port index on the Driver Station that the controller is plugged into (0-5).
     */
    private TigerPad(final int port) {
        super(port);
        dialOffset = 0.0;
        HAL.report(tResourceType.kResourceType_Controller, port + 1);
    }

    /**
     * Get the lower arm axis value of the controller.
     *
     * @return The axis value.
     */
    public double getLowerArm() {
        return getRawAxis(Axis.LowerArm.value);
    }

    /**
     * Get the upper arm axis value of the controller.
     *
     * @return The axis value.
     */
    public double getUpperArm() {
        return getRawAxis(Axis.UpperArm.value);
    }

    /**
     * Get the wrist pivot axis value of the controller.
     *
     * @return The axis value.
     */
    public double getWristPivot() {
        return getRawAxis(Axis.WristPivot.value);
    }

    /**
     * Sets the dial offset for the wrist rotation to the current raw axis value.
     */
    public void zeroWristDial() {
        dialOffset = getRawAxis(Axis.WristRotation.value);
    }

    /**
     * Gets the wrist rotation in degrees.
     * 
     * This method retrieves the raw axis value for wrist rotation, adjusts it by
     * subtracting the dial offset, and then maps the adjusted value from the range
     * [-0.5, 0.5] to the range [-180.0, 180.0] degrees.
     * 
     * <p>To get the unitless wrist dial value without offset, use
     * {@link GenericHID#getRawAxis(int) getRawAxis(Axis.WristRotation.value)}.
     * 
     * @return The wrist rotation in degrees, ranging from -180.0 to 180.0.
     */
    public double getWristDegrees() {
        var adjustedValue = getRawAxis(Axis.WristRotation.value) - dialOffset;
        return Utils.mapRange(adjustedValue, -0.5, 0.5, -180.0, 180.0);
    }

    /**
     * Gets the wrist rotation in radians.
     *
     * This method retrieves the raw axis value for wrist rotation, adjusts it by subtracting
     * the dial offset, and then maps the adjusted value from the range [-0.5, 0.5] to the range
     * [-π, π].
     * 
     * <p>To get the unitless wrist dial value without offset, use
     * {@link GenericHID#getRawAxis(int) getRawAxis(Axis.WristRotation.value)}.
     *
     * @return The wrist rotation in radians -π to π.
     */
    public double getWristRadians() {
        var adjustedValue = getRawAxis(Axis.WristRotation.value) - dialOffset;
        return Utils.mapRange(adjustedValue, -0.5, 0.5, -Math.PI, Math.PI);
    }

    /**
     * Get the mode of the LED.
     *
     * @param button The {@link LED} to be read
     * @return The {@link LEDMode} of the LED.
     */
    public LEDMode getLEDMode(LED led) {
        var modeValue = (int) led.entry.get();
        return LEDMode.getByValue(modeValue);
    }

    /**
     * Set the mode of the LED.
     *
     * @param led The {@link LED} to be set.
     * @param mode The {@link LEDMode} to set the LED to.
     */
    public void setLEDMode(LED led, LEDMode mode) {
        led.entry.set(mode.value);
    }

    /**
     * Set the mode of all LEDs.
     *
     * @param mode The {@link LEDMode} to set the LED to.
     */
    public void setAllLEDs(LEDMode mode) {
        for (LED led : LED.values()) {
            setLEDMode(led, mode);
        }
    }

    /**
     * Read the value of the intake button on the controller.
     *
     * @return The state of the button.
     */
    public boolean getIntakeButton() {
        return getRawButton(Button.Intake.value);
    }

    /**
     * Whether the intake button was pressed since the last check.
     * 
     * @return Whether the button was pressed since the last check.
     */
    public boolean getIntakeButtonPressed() {
        return getRawButtonPressed(Button.Intake.value);
    }

    /**
     * Whether the intake button was released since the last check.
     * 
     * @return Whether the button was released since the last check.
     */
    public boolean getIntakeButtonReleased() {
        return getRawButtonReleased(Button.Intake.value);
    }

    /**
     * Constructs an event instance around the intake button's digital signal.
     * 
     * @param loop the event loop instance to attach the event to.
     * @return an event instance representing the intake button's digital signal
     *     attached to the given loop.
     */
    public BooleanEvent intake(EventLoop loop) {
        return button(Button.Intake.value, loop);
    }

    /**
     * Read the mode of the intake button LED on the controller.
     * 
     * @return The mode of the button's LED.
     */
    public LEDMode getIntakeButtonLEDMode() {
        return getLEDMode(LED.Intake);
    }

    
    /**
     * Set the mode of the intake button LED.
     * 
     * @param mode The {@link LEDMode} to set the button's LED to.
     */
    public void setIntakeButtonLEDMode(LEDMode mode) {
        setLEDMode(LED.Intake, mode);
    }

    /**
     * Read the value of the level 1 button on the controller.
     *
     * @return The state of the button.
     */
    public boolean getLevel1Button() {
        return getRawButton(Button.Level1.value);
    }

    /**
     * Whether the level 1 button was pressed since the last check.
     * 
     * @return Whether the button was pressed since the last check.
     */
    public boolean getLevel1ButtonPressed() {
        return getRawButtonPressed(Button.Level1.value);
    }

    /**
     * Whether the level 1 button was released since the last check.
     * 
     * @return Whether the button was released since the last check.
     */
    public boolean getLevel1ButtonReleased() {
        return getRawButtonReleased(Button.Level1.value);
    }

    /**
     * Constructs an event instance around the level 1 button's digital signal.
     * 
     * @param loop the event loop instance to attach the event to.
     * @return an event instance representing the level 1 button's digital signal
     *     attached to the given loop.
     */
    public BooleanEvent level1(EventLoop loop) {
        return button(Button.Level1.value, loop);
    }

    /**
     * Read the mode of the level 1 button LED on the controller.
     * 
     * @return The mode of the button's LED.
     */
    public LEDMode getLevel1ButtonLEDMode() {
        return getLEDMode(LED.Level1);
    }

    /**
     * Set the mode of the level 1 button LED.
     * 
     * @param mode The {@link LEDMode} to set the button's LED to.
     */
    public void setLevel1ButtonLEDMode(LEDMode mode) {
        setLEDMode(LED.Level1, mode);
    }

    /**
     * Read the value of the level 2 left button on the controller.
     *
     * @return The state of the button.
     */
    public boolean getLevel2LeftButton() {
        return getRawButton(Button.Level2Left.value);
    }

    /**
     * Whether the level 2 left button was pressed since the last check.
     * 
     * @return Whether the button was pressed since the last check.
     */
    public boolean getLevel2LeftButtonPressed() {
        return getRawButtonPressed(Button.Level2Left.value);
    }

    /**
     * Whether the level 2 left button was released since the last check.
     * 
     * @return Whether the button was released since the last check.
     */
    public boolean getLevel2LeftButtonReleased() {
        return getRawButtonReleased(Button.Level2Left.value);
    }

    /**
     * Constructs an event instance around the level 2 left button's digital signal.
     * 
     * @param loop the event loop instance to attach the event to.
     * @return an event instance representing the level 2 left button's digital signal
     *     attached to the given loop.
     */
    public BooleanEvent level2Left(EventLoop loop) {
        return button(Button.Level2Left.value, loop);
    }

    /**
     * Read the mode of the level 2 left button LED on the controller.
     * 
     * @return The mode of the button's LED.
     */
    public LEDMode getLevel2LeftButtonLEDMode() {
        return getLEDMode(LED.Level2Left);
    }

    /**
     * Set the mode of the level 2 left button LED.
     * 
     * @param mode The {@link LEDMode} to set the button's LED to.
     */
    public void setLevel2LeftButtonLEDMode(LEDMode mode) {
        setLEDMode(LED.Level2Left, mode);
    }

    /**
     * Read the value of the level 2 right button on the controller.
     *
     * @return The state of the button.
     */
    public boolean getLevel2RightButton() {
        return getRawButton(Button.Level2Right.value);
    }

    /**
     * Whether the level 2 right button was pressed since the last check.
     * 
     * @return Whether the button was pressed since the last check.
     */
    public boolean getLevel2RightButtonPressed() {
        return getRawButtonPressed(Button.Level2Right.value);
    }

    /**
     * Whether the level 2 right button was released since the last check.
     * 
     * @return Whether the button was released since the last check.
     */
    public boolean getLevel2RightButtonReleased() {
        return getRawButtonReleased(Button.Level2Right.value);
    }

    /**
     * Constructs an event instance around the level 2 right button's digital signal.
     * 
     * @param loop the event loop instance to attach the event to.
     * @return an event instance representing the level 2 right button's digital signal
     *     attached to the given loop.
     */
    public BooleanEvent level2Right(EventLoop loop) {
        return button(Button.Level2Right.value, loop);
    }

    /**
     * Read the mode of the level 2 right button LED on the controller.
     * 
     * @return The mode of the button's LED.
     */
    public LEDMode getLevel2RightButtonLEDMode() {
        return getLEDMode(LED.Level2Right);
    }

    /**
     * Set the mode of the level 2 right button LED.
     * 
     * @param mode The {@link LEDMode} to set the button's LED to.
     */
    public void setLevel2RightButtonLEDMode(LEDMode mode) {
        setLEDMode(LED.Level2Right, mode);
    }

    /**
     * Read the value of the level 3 left button on the controller.
     *
     * @return The state of the button.
     */
    public boolean getLevel3LeftButton() {
        return getRawButton(Button.Level3Left.value);
    }

    /**
     * Whether the level 3 left button was pressed since the last check.
     * 
     * @return Whether the button was pressed since the last check.
     */
    public boolean getLevel3LeftButtonPressed() {
        return getRawButtonPressed(Button.Level3Left.value);
    }

    /**
     * Whether the level 3 left button was released since the last check.
     * 
     * @return Whether the button was released since the last check.
     */
    public boolean getLevel3LeftButtonReleased() {
        return getRawButtonReleased(Button.Level3Left.value);
    }

    /**
     * Constructs an event instance around the level 3 left button's digital signal.
     * 
     * @param loop the event loop instance to attach the event to.
     * @return an event instance representing the level 3 left button's digital signal
     *     attached to the given loop.
     */
    public BooleanEvent level3Left(EventLoop loop) {
        return button(Button.Level3Left.value, loop);
    }

    /**
     * Read the mode of the level 3 left button LED on the controller.
     * 
     * @return The mode of the button's LED.
     */
    public LEDMode getLevel3LeftButtonLEDMode() {
        return getLEDMode(LED.Level3Left);
    }

    /**
     * Set the mode of the level 3 left button LED.
     * 
     * @param mode The {@link LEDMode} to set the button's LED to.
     */
    public void setLevel3LeftButtonLEDMode(LEDMode mode) {
        setLEDMode(LED.Level3Left, mode);
    }

    /**
     * Read the value of the level 3 right button on the controller.
     *
     * @return The state of the button.
     */
    public boolean getLevel3RightButton() {
        return getRawButton(Button.Level3Right.value);
    }

    /**
     * Whether the level 3 right button was pressed since the last check.
     * 
     * @return Whether the button was pressed since the last check.
     */
    public boolean getLevel3RightButtonPressed() {
        return getRawButtonPressed(Button.Level3Right.value);
    }

    /**
     * Whether the level 3 right button was released since the last check.
     * 
     * @return Whether the button was released since the last check.
     */
    public boolean getLevel3RightButtonReleased() {
        return getRawButtonReleased(Button.Level3Right.value);
    }

    /**
     * Constructs an event instance around the level 3 right button's digital signal.
     * 
     * @param loop the event loop instance to attach the event to.
     * @return an event instance representing the level 3 right button's digital signal
     *     attached to the given loop.
     */
    public BooleanEvent level3Right(EventLoop loop) {
        return button(Button.Level3Right.value, loop);
    }

    /**
     * Read the mode of the level 3 right button LED on the controller.
     * 
     * @return The mode of the button's LED.
     */
    public LEDMode getLevel3RightButtonLEDMode() {
        return getLEDMode(LED.Level3Right);
    }

    /**
     * Set the mode of the level 3 right button LED.
     * 
     * @param mode The {@link LEDMode} to set the button's LED to.
     */
    public void setLevel3RightButtonLEDMode(LEDMode mode) {
        setLEDMode(LED.Level3Right, mode);
    }

    /**
     * Read the value of the level 4 left button on the controller.
     *
     * @return The state of the button.
     */
    public boolean getLevel4LeftButton() {
        return getRawButton(Button.Level4Left.value);
    }

    /**
     * Whether the level 4 left button was pressed since the last check.
     * 
     * @return Whether the button was pressed since the last check.
     */
    public boolean getLevel4LeftButtonPressed() {
        return getRawButtonPressed(Button.Level4Left.value);
    }

    /**
     * Whether the level 4 left button was released since the last check.
     * 
     * @return Whether the button was released since the last check.
     */
    public boolean getLevel4LeftButtonReleased() {
        return getRawButtonReleased(Button.Level4Left.value);
    }

    /**
     * Constructs an event instance around the level 4 left button's digital signal.
     * 
     * @param loop the event loop instance to attach the event to.
     * @return an event instance representing the level 4 left button's digital signal
     *     attached to the given loop.
     */
    public BooleanEvent level4Left(EventLoop loop) {
        return button(Button.Level4Left.value, loop);
    }

    /**
     * Read the mode of the level 4 left button LED on the controller.
     * 
     * @return The mode of the button's LED.
     */
    public LEDMode getLevel4LeftButtonLEDMode() {
        return getLEDMode(LED.Level4Left);
    }

    /**
     * Set the mode of the level 4 left button LED.
     * 
     * @param mode The {@link LEDMode} to set the button's LED to.
     */
    public void setLevel4LeftButtonLEDMode(LEDMode mode) {
        setLEDMode(LED.Level4Left, mode);
    }

    /**
     * Read the value of the level 4 right button on the controller.
     *
     * @return The state of the button.
     */
    public boolean getLevel4RightButton() {
        return getRawButton(Button.Level4Right.value);
    }

    /**
     * Whether the level 4 right button was pressed since the last check.
     * 
     * @return Whether the button was pressed since the last check.
     */
    public boolean getLevel4RightButtonPressed() {
        return getRawButtonPressed(Button.Level4Right.value);
    }

    /**
     * Whether the level 4 right button was released since the last check.
     * 
     * @return Whether the button was released since the last check.
     */
    public boolean getLevel4RightButtonReleased() {
        return getRawButtonReleased(Button.Level4Right.value);
    }

    /**
     * Constructs an event instance around the level 4 right button's digital signal.
     * 
     * @param loop the event loop instance to attach the event to.
     * @return an event instance representing the level 4 right button's digital signal
     *     attached to the given loop.
     */
    public BooleanEvent level4Right(EventLoop loop) {
        return button(Button.Level4Right.value, loop);
    }

    /**
     * Read the mode of the level 4 right button LED on the controller.
     * 
     * @return The mode of the button's LED.
     */
    public LEDMode getLevel4RightButtonLEDMode() {
        return getLEDMode(LED.Level4Right);
    }

    /**
     * Set the mode of the level 4 right button LED.
     * 
     * @param mode The {@link LEDMode} to set the button's LED to.
     */
    public void setLevel4RightButtonLEDMode(LEDMode mode) {
        setLEDMode(LED.Level4Right, mode);
    }

    /**
     * Read the value of the arm override switch on the controller.
     *
     * @return The state of the switch.
     */
    public boolean getArmOverrideSwitch() {
        return getRawButton(ToggleSwitch.ArmOverride.value);
    }

    /**
     * Whether the arm override switch was enabled since the last check.
     * 
     * @return Whether the switch was enabled since the last check.
     */
    public boolean getArmOverrideSwitchEnabled() {
        return getRawButtonPressed(ToggleSwitch.ArmOverride.value);
    }

    /**
     * Whether the arm override switch was disabled since the last check.
     * 
     * @return Whether the switch was disabled since the last check.
     */
    public boolean getArmOverrideSwitchDisabled() {
        return getRawButtonReleased(ToggleSwitch.ArmOverride.value);
    }

    /**
     * Constructs an event instance around the arm override switch's digital signal.
     * 
     * @param loop the event loop instance to attach the event to.
     * @return an event instance representing the arm override switch's digital signal
     *     attached to the given loop.
     */
    public BooleanEvent armOverride(EventLoop loop) {
        return button(ToggleSwitch.ArmOverride.value, loop);
    }

    /**
     * Read the value of the wrist override switch on the controller.
     *
     * @return The state of the switch.
     */
    public boolean getWristOverrideSwitch() {
        return getRawButton(ToggleSwitch.WristOverride.value);
    }

    /**
     * Whether the wrist override switch was enabled since the last check.
     * 
     * @return Whether the switch was enabled since the last check.
     */
    public boolean getWristOverrideSwitchEnabled() {
        return getRawButtonPressed(ToggleSwitch.WristOverride.value);
    }

    /**
     * Whether the wrist override switch was disabled since the last check.
     * 
     * @return Whether the switch was disabled since the last check.
     */
    public boolean getWristOverrideSwitchDisabled() {
        return getRawButtonReleased(ToggleSwitch.WristOverride.value);
    }

    /**
     * Constructs an event instance around the wrist override switch's digital signal.
     * 
     * @param loop the event loop instance to attach the event to.
     * @return an event instance representing the wrist override switch's digital signal
     *     attached to the given loop.
     */
    public BooleanEvent wristOverride(EventLoop loop) {
        return button(ToggleSwitch.WristOverride.value, loop);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("HID");
        builder.publishConstString("ControllerType", "TigerPad");
        builder.addDoubleProperty("LowerArm", this::getLowerArm, null);
        builder.addDoubleProperty("UpperArm", this::getUpperArm, null);
        builder.addDoubleProperty("WristPivot", this::getWristPivot, null);
        builder.addBooleanProperty("Intake", this::getIntakeButton, null);
        builder.addStringProperty(
            "IntakeLED",
            () -> getIntakeButtonLEDMode().toString(),
            (String name) -> setIntakeButtonLEDMode(LEDMode.safeValueOf(name))
        );
        builder.addBooleanProperty("Level1", this::getLevel1Button, null);
        builder.addStringProperty(
            "Level1LED",
            () -> getLevel1ButtonLEDMode().toString(),
            (String name) -> setLevel1ButtonLEDMode(LEDMode.safeValueOf(name))
        );
        builder.addBooleanProperty("Level2Left", this::getLevel2LeftButton, null);
        builder.addStringProperty(
            "Level2LeftLED",
            () -> getLevel2LeftButtonLEDMode().toString(),
            (String name) -> setLevel2LeftButtonLEDMode(LEDMode.safeValueOf(name))
        );
        builder.addBooleanProperty("Level2Right", this::getLevel2RightButton, null);
        builder.addStringProperty(
            "Level2RightLED",
            () -> getLevel2RightButtonLEDMode().toString(),
            (String name) -> setLevel2RightButtonLEDMode(LEDMode.safeValueOf(name))
        );
        builder.addBooleanProperty("Level3Left", this::getLevel3LeftButton, null);
        builder.addStringProperty(
            "Level3LeftLED",
            () -> getLevel3LeftButtonLEDMode().toString(),
            (String name) -> setLevel3LeftButtonLEDMode(LEDMode.safeValueOf(name))
        );
        builder.addBooleanProperty("Level3Right", this::getLevel3RightButton, null);
        builder.addStringProperty(
            "Level3RightLED",
            () -> getLevel3RightButtonLEDMode().toString(),
            (String name) -> setLevel3RightButtonLEDMode(LEDMode.safeValueOf(name))
        );
        builder.addBooleanProperty("Level4Left", this::getLevel4LeftButton, null);
        builder.addStringProperty(
            "Level4LeftLED",
            () -> getLevel4LeftButtonLEDMode().toString(),
            (String name) -> setLevel4LeftButtonLEDMode(LEDMode.safeValueOf(name))
        );
        builder.addBooleanProperty("Level4Right", this::getLevel4RightButton, null);
        builder.addStringProperty(
            "Level4RightLED",
            () -> getLevel4RightButtonLEDMode().toString(),
            (String name) -> setLevel4RightButtonLEDMode(LEDMode.safeValueOf(name))
        );
        builder.addBooleanProperty("ArmOverride", this::getArmOverrideSwitch, null);
        builder.addBooleanProperty("WristOverride", this::getWristOverrideSwitch, null);
    }

    @Override
    public int getPOV(int pov) {
        throw new UnsupportedOperationException("POV methods are not supported by TigerPad.");
    }

    @Override
    public int getPOV() {
        throw new UnsupportedOperationException("POV methods are not supported by TigerPad.");
    }

    @Override
    public BooleanEvent pov(int angle, EventLoop loop) {
        throw new UnsupportedOperationException("POV methods are not supported by TigerPad.");
    }

    @Override
    public BooleanEvent pov(int pov, int angle, EventLoop loop) {
        throw new UnsupportedOperationException("POV methods are not supported by TigerPad.");
    }

    @Override
    public BooleanEvent povUp(EventLoop loop) {
        throw new UnsupportedOperationException("POV methods are not supported by TigerPad.");
    }

    @Override
    public BooleanEvent povUpLeft(EventLoop loop) {
        throw new UnsupportedOperationException("POV methods are not supported by TigerPad.");
    }

    @Override
    public BooleanEvent povUpRight(EventLoop loop) {
        throw new UnsupportedOperationException("POV methods are not supported by TigerPad.");
    }

    @Override
    public BooleanEvent povDown(EventLoop loop) {
        throw new UnsupportedOperationException("POV methods are not supported by TigerPad.");
    }

    @Override
    public BooleanEvent povLeft(EventLoop loop) {
        throw new UnsupportedOperationException("POV methods are not supported by TigerPad.");
    }

    @Override
    public BooleanEvent povRight(EventLoop loop) {
        throw new UnsupportedOperationException("POV methods are not supported by TigerPad.");
    }

    @Override
    public BooleanEvent povDownLeft(EventLoop loop) {
        throw new UnsupportedOperationException("POV methods are not supported by TigerPad.");
    }

    @Override
    public BooleanEvent povDownRight(EventLoop loop) {
        throw new UnsupportedOperationException("POV methods are not supported by TigerPad.");
    }

    @Override
    public BooleanEvent povCenter(EventLoop loop) {
        throw new UnsupportedOperationException("POV methods are not supported by TigerPad.");
    }

    @Override
    public void setRumble(RumbleType type, double value) {
        throw new UnsupportedOperationException("Rumble is not supported by TigerPad.");
    }
}
