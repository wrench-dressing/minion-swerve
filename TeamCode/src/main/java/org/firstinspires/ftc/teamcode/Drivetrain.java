package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

public class Drivetrain {
    private class Module {
        public final DcMotorEx top;
        public final DcMotorEx bottom;

        public Module(DcMotorEx top, DcMotorEx bottom) {
            this.top = top;
            this.bottom = bottom;
        }

        public void go(double translation, double angle, boolean fieldRelative) {
            double rotPower = rotationalPower(angle, fieldRelative);
            double speedPower = translation * maxSpeed();

            top.setPower(rotPower + speedPower);
            bottom.setPower(rotPower - speedPower);
        }

        public void stop() {
            top.setPower(0);
            bottom.setPower(0);
        }

        public double encoder() {
            return (top.getCurrentPosition() / 8192.0 + 1) % 1;
        }

        private double rotationalPower(double toAngleDeg, boolean fieldRelative) {
            double rot = (toAngleDeg + 360) % 360 / 360;
            if (fieldRelative) {
                double currentHeading = (imu.getRobotYawPitchRollAngles().getYaw() / 360 + 0.5) % 1;
                rot -= currentHeading;
            }

            double rotChange = ((encoder() - rot + 0.5) % 1 + 1) % 1 - 0.5;
            double rotClamp = 1 - maxSpeed();
            return Utils.clamp(-rotClamp, rotClamp, rotChange * 5) % 1;
        }
    }

    private final Module front;
    private final Module back;
    private boolean fastMode = false;
    private final IMU imu;

    public void setFastMode(boolean fastMode) {
        this.fastMode = fastMode;
    }

    private double maxSpeed() {
        return fastMode ? Configuration.maxSpeedHigh : Configuration.maxSpeedLow;
    }

    public Drivetrain(HardwareMap hw, IMU imu) {
        front = new Module(
                hw.get(DcMotorEx.class, Configuration.FRONT_TOP_MOTOR),
                hw.get(DcMotorEx.class, Configuration.FRONT_BOTTOM_MOTOR));
        back = new Module(
                hw.get(DcMotorEx.class, Configuration.BACK_TOP_MOTOR),
                hw.get(DcMotorEx.class, Configuration.BACK_BOTTOM_MOTOR));
        this.imu = imu;
    }

    public void move(double translation, double angle) {
        front.go(translation, angle, true);
        back.go(-translation, angle, true);
    }

    public void turn(double rps) {
        front.go(rps * Configuration.turnSpeed, 90, false);
        back.go(rps * Configuration.turnSpeed, 90, false);
    }

    public void stop() {
        front.stop();
        back.stop();
    }
}
