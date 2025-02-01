package frc.robot.utils;

import java.util.HashMap;
import java.util.Locale;
import java.util.Map;
import java.util.NoSuchElementException;
import java.util.Set;

import org.apache.commons.lang3.BitField;
import org.apache.commons.lang3.StringUtils;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.event.BooleanEvent;
import edu.wpi.first.wpilibj.event.EventLoop;

public class TigerPad extends GenericHID implements Sendable {
    
    private int outputState;

    public enum Button {
        Intake(0),
        Level1(1),
        Level2Left(2),
        Level2Right(3),
        Level3Left(4),
        Level3Right(5),
        Level4Left(6),
        Level4Right(7);

        public final int value;

        Button(int value) {
            this.value = value;
        }

        @Override
        public String toString() {
            return this.name() + "Button";
        }
    }

    public enum ToggleSwitch {
        ArmOverride(8),
        WristOverride(9);

        public final int value;

        ToggleSwitch(int value) {
            this.value = value;
        }

        @Override
        public String toString() {
            return this.name() + "Switch";
        }
    }

    public enum Axis {
        LowerArm(0),
        UpperArm(1),
        WristPivot(2);

        public final int value;

        Axis(int value) {
            this.value = value;
        }

        @Override
        public String toString() {
            return this.name() + "Axis";
        }
    }

    public enum LED {
        Intake(0),
        Level1(1),
        Level2Left(2),
        Level2Right(3),
        Level3Left(4),
        Level3Right(5),
        Level4Left(6),
        Level4Right(7);

        private static final Map<LED, BitField> bitFieldMap = new HashMap<>();

        static {
            for (LED led : LED.values()) {
                bitFieldMap.put(led, new BitField(0b11 << (led.value * 2)));
            }
        }

        public final int value;

        LED(int value) {
            this.value = value;
        }

        private BitField getBitField() {
            return bitFieldMap.get(this);
        }

        @Override
        public String toString() {
            return this.name() + "LED";
        }
    }

    public enum LEDMode {
        Off(0),
        Blink(1),
        On(2);

        private static final Map<Integer, LEDMode> ledsByValue = new HashMap<>();

        static {
            for (LEDMode mode : LEDMode.values()) {
                ledsByValue.put(mode.value, mode);
            }
        }

        private static LEDMode getByValue(int value) {
            if (value - 1 < ledsByValue.size()) {
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

        public final int value;

        LEDMode(int value) {
            this.value = value;
        }

        @Override
        public String toString() {
            if (this == Blink) {
                return this.name().toLowerCase() + "ing";
            }

            return this.name().toLowerCase();
        }
    }

    public TigerPad(final int port) {
        super(port);
        outputState = 0;
        HAL.report(tResourceType.kResourceType_Controller, port + 1);
    }

    public double getLowerArm() {
        return getRawAxis(Axis.LowerArm.value);
    }

    public double getUpperArm() {
        return getRawAxis(Axis.UpperArm.value);
    }

    public double getWristPivot() {
        return getRawAxis(Axis.WristPivot.value);
    }

    public LEDMode getLEDMode(LED led) {
        var modeValue = (outputState >> (led.value * 2)) & 0b11;
        return LEDMode.getByValue(modeValue);
    }

    public void setLEDMode(LED led, LEDMode mode) {
        if (mode == null) {
            return;
        }

        var bitField = led.getBitField();
        outputState = bitField.setValue(outputState, mode.value);
        setOutputs(outputState);
    }

    public boolean getIntakeButton() {
        return getRawButton(Button.Intake.value);
    }

    public boolean getIntakeButtonPressed() {
        return getRawButtonPressed(Button.Intake.value);
    }

    public boolean getIntakeButtonReleased() {
        return getRawButtonReleased(Button.Intake.value);
    }

    public BooleanEvent intake(EventLoop loop) {
        return button(Button.Intake.value, loop);
    }

    public LEDMode getIntakeButtonLEDMode() {
        return getLEDMode(LED.Intake);
    }

    public void setIntakeButtonLEDMode(LEDMode mode) {
        setLEDMode(LED.Intake, mode);
    }

    public boolean getLevel1Button() {
        return getRawButton(Button.Level1.value);
    }

    public boolean getLevel1ButtonPressed() {
        return getRawButtonPressed(Button.Level1.value);
    }

    public boolean getLevel1ButtonReleased() {
        return getRawButtonReleased(Button.Level1.value);
    }

    public LEDMode getLevel1ButtonLEDMode() {
        return getLEDMode(LED.Level1);
    }

    public void setLevel1ButtonLEDMode(LEDMode mode) {
        setLEDMode(LED.Level1, mode);
    }

    public BooleanEvent level1(EventLoop loop) {
        return button(Button.Level1.value, loop);
    }

    public boolean getLevel2LeftButton() {
        return getRawButton(Button.Level2Left.value);
    }

    public boolean getLevel2LeftButtonPressed() {
        return getRawButtonPressed(Button.Level2Left.value);
    }

    public boolean getLevel2LeftButtonReleased() {
        return getRawButtonReleased(Button.Level2Left.value);
    }

    public LEDMode getLevel2LeftButtonLEDMode() {
        return getLEDMode(LED.Level2Left);
    }

    public void setLevel2LeftButtonLEDMode(LEDMode mode) {
        setLEDMode(LED.Level2Left, mode);
    }

    public BooleanEvent level2Left(EventLoop loop) {
        return button(Button.Level2Left.value, loop);
    }

    public boolean getLevel2RightButton() {
        return getRawButton(Button.Level2Right.value);
    }

    public boolean getLevel2RightButtonPressed() {
        return getRawButtonPressed(Button.Level2Right.value);
    }

    public boolean getLevel2RightButtonReleased() {
        return getRawButtonReleased(Button.Level2Right.value);
    }

    public LEDMode getLevel2RightButtonLEDMode() {
        return getLEDMode(LED.Level2Right);
    }

    public void setLevel2RightButtonLEDMode(LEDMode mode) {
        setLEDMode(LED.Level2Right, mode);
    }

    public BooleanEvent level2Right(EventLoop loop) {
        return button(Button.Level2Right.value, loop);
    }

    public boolean getLevel3LeftButton() {
        return getRawButton(Button.Level3Left.value);
    }

    public boolean getLevel3LeftButtonPressed() {
        return getRawButtonPressed(Button.Level3Left.value);
    }

    public boolean getLevel3LeftButtonReleased() {
        return getRawButtonReleased(Button.Level3Left.value);
    }

    public LEDMode getLevel3LeftButtonLEDMode() {
        return getLEDMode(LED.Level3Left);
    }

    public void setLevel3LeftButtonLEDMode(LEDMode mode) {
        setLEDMode(LED.Level3Left, mode);
    }

    public BooleanEvent level3Left(EventLoop loop) {
        return button(Button.Level3Left.value, loop);
    }

    public boolean getLevel3RightButton() {
        return getRawButton(Button.Level3Right.value);
    }

    public boolean getLevel3RightButtonPressed() {
        return getRawButtonPressed(Button.Level3Right.value);
    }

    public boolean getLevel3RightButtonReleased() {
        return getRawButtonReleased(Button.Level3Right.value);
    }

    public LEDMode getLevel3RightButtonLEDMode() {
        return getLEDMode(LED.Level3Right);
    }

    public void setLevel3RightButtonLEDMode(LEDMode mode) {
        setLEDMode(LED.Level3Right, mode);
    }

    public BooleanEvent level3Right(EventLoop loop) {
        return button(Button.Level3Right.value, loop);
    }

    public boolean getLevel4LeftButton() {
        return getRawButton(Button.Level4Left.value);
    }

    public boolean getLevel4LeftButtonPressed() {
        return getRawButtonPressed(Button.Level4Left.value);
    }

    public boolean getLevel4LeftButtonReleased() {
        return getRawButtonReleased(Button.Level4Left.value);
    }

    public LEDMode getLevel4LeftButtonLEDMode() {
        return getLEDMode(LED.Level4Left);
    }

    public void setLevel4LeftButtonLEDMode(LEDMode mode) {
        setLEDMode(LED.Level4Left, mode);
    }

    public BooleanEvent level4Left(EventLoop loop) {
        return button(Button.Level4Left.value, loop);
    }

    public boolean getLevel4RightButton() {
        return getRawButton(Button.Level4Right.value);
    }

    public boolean getLevel4RightButtonPressed() {
        return getRawButtonPressed(Button.Level4Right.value);
    }

    public boolean getLevel4RightButtonReleased() {
        return getRawButtonReleased(Button.Level4Right.value);
    }

    public LEDMode getLevel4RightButtonLEDMode() {
        return getLEDMode(LED.Level4Right);
    }

    public void setLevel4RightButtonLEDMode(LEDMode mode) {
        setLEDMode(LED.Level4Right, mode);
    }

    public BooleanEvent level4Right(EventLoop loop) {
        return button(Button.Level4Right.value, loop);
    }

    public boolean getArmOverrideSwitch() {
        return getRawButton(ToggleSwitch.ArmOverride.value);
    }

    public boolean getArmOverrideSwitchEnabled() {
        return getRawButtonPressed(ToggleSwitch.ArmOverride.value);
    }

    public boolean getArmOverrideSwitchDisabled() {
        return getRawButtonReleased(ToggleSwitch.ArmOverride.value);
    }

    public BooleanEvent armOverride(EventLoop loop) {
        return button(ToggleSwitch.ArmOverride.value, loop);
    }

    public boolean getWristOverrideSwitch() {
        return getRawButton(ToggleSwitch.WristOverride.value);
    }

    public boolean getWristOverrideSwitchEnabled() {
        return getRawButtonPressed(ToggleSwitch.WristOverride.value);
    }

    public boolean getWristOverrideSwitchDisabled() {
        return getRawButtonReleased(ToggleSwitch.WristOverride.value);
    }

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
}
