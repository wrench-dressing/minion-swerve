package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

public class Drivetrain {
    private class Module {
        public final DcMotorEx top;
        public final DcMotorEx bottom;

        private final Ramp speedCommandRamp;

        public Module(DcMotorEx top, DcMotorEx bottom) {
            this.top = top;
            this.bottom = bottom;
            speedCommandRamp = new Ramp(0, Configuration.maxAccelRps);
        }

        public void go(double translation, double angle, boolean fieldRelative) {
            double desiredRotation = calculateDesiredRotation(angle, fieldRelative);

            // more efficient to invert the wheels
            boolean invert = Math.abs(desiredRotation) > 0.25;
            if (invert)
                desiredRotation = calculateDesiredRotation(angle + 180, fieldRelative);

            double desiredSpeed;
            if (Math.abs(desiredRotation) < Configuration.rotToleranceToDrive)
                desiredSpeed = translation * Configuration.maxSpeed;
            else
                desiredSpeed = 0;

            double maxRotPower = 1 - desiredSpeed;
            double rotPower = Utils.clamp(-maxRotPower, maxRotPower, desiredRotation * Configuration.rotationGain);

            double desiredSpeedCommand = desiredSpeed * (invert ? -1 : 1);
            double speedCommand = speedCommandRamp.update(desiredSpeedCommand);

            top.setPower(rotPower + speedCommand);
            bottom.setPower(rotPower - speedCommand);
        }

        public void stop() {
            top.setPower(0);
            bottom.setPower(0);
            speedCommandRamp.update(0);
        }

        public double encoder() {
            return (top.getCurrentPosition() / 8192.0 + 1) % 1;
        }

        private double calculateDesiredRotation(double toAngleDeg, boolean fieldRelative) {
            double rot = (toAngleDeg + 360) % 360 / 360;
            if (fieldRelative) {
                double currentHeading = (imu.getRobotYawPitchRollAngles().getYaw() / 360 + 0.5) % 1;
                rot -= currentHeading;
            }

            // shortest wraparound distance to point
            // it would be simpler if Java moduli always returned positive
            // so there's a lot of normalization math
            return ((encoder() - rot + 0.5) % 1 + 1) % 1 - 0.5;
        }
    }

    private final Module front;
    private final Module back;
    private final IMU imu;

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
