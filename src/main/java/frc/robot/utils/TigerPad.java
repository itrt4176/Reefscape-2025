package frc.robot.utils;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.GenericHID;

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

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("HID");
        builder.publishConstString("ControllerType", "TigerPad");
    }
}
