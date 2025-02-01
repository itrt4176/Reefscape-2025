package frc.robot.utils;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.event.BooleanEvent;
import edu.wpi.first.wpilibj.event.EventLoop;

public class TigerPad extends GenericHID implements Sendable {
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

    public TigerPad(final int port) {
        super(port);
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

    public boolean getLevel1Button() {
        return getRawButton(Button.Level1.value);
    }

    public boolean getLevel1ButtonPressed() {
        return getRawButtonPressed(Button.Level1.value);
    }

    public boolean getLevel1ButtonReleased() {
        return getRawButtonReleased(Button.Level1.value);
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
        builder.addBooleanProperty("Level1", this::getLevel1Button, null);
        builder.addBooleanProperty("Level2Left", this::getLevel2LeftButton, null);
        builder.addBooleanProperty("Level2Right", this::getLevel2RightButton, null);
        builder.addBooleanProperty("Level3Left", this::getLevel3LeftButton, null);
        builder.addBooleanProperty("Level3Right", this::getLevel3RightButton, null);
        builder.addBooleanProperty("Level4Left", this::getLevel4LeftButton, null);
        builder.addBooleanProperty("Level4Right", this::getLevel4RightButton, null);
        builder.addBooleanProperty("ArmOverride", this::getArmOverrideSwitch, null);
        builder.addBooleanProperty("WristOverride", this::getWristOverrideSwitch, null);
    }
}
