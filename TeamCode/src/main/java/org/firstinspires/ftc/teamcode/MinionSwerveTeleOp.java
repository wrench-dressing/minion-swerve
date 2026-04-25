package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;

@SuppressWarnings("unused")
@TeleOp(name = "Minion Swerve TeleOp")
public class MinionSwerveTeleOp extends LinearOpMode {
    Drivetrain drive;
    IMU imu;

    boolean fastMode = false;

    @Override
    public void runOpMode() {
        waitForStart();

        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.RIGHT, RevHubOrientationOnRobot.UsbFacingDirection.UP)));
        imu.resetYaw();
        drive = new Drivetrain(hardwareMap, imu);

        while (opModeIsActive()) {
            telemetry.addData("Instructions", "Left joy for translation." +
                    "Triggers to turn. B = toggle Fast Mode." +
                    "Options = Reset encoders. Start = Reset IMU.");

            if (gamepad1.bWasPressed()) {
                fastMode = !fastMode;
                drive.setFastMode(fastMode);
            }
            telemetry.addData("Fast mode", fastMode ? "ON" : "OFF");

            if (gamepad1.startWasPressed())
                imu.resetYaw();

            movement();

            telemetry.update();
        }
    }

    private void movement() {
        double turnLeft = gamepad1.left_trigger;
        double turnRight = gamepad1.right_trigger;

        if (turnLeft > 0.1 || turnRight > 0.1) {
            // turning
            double amount = turnLeft - turnRight;
            drive.turn(amount);
            return;
        }

        // regular movement
        Vector leftJoy = new Vector(gamepad1.left_stick_x, gamepad1.left_stick_y - 0.01).deadbanded(0.1);
        telemetry.addData("left joystick", leftJoy);
        if (!leftJoy.isZeroed()) {
            drive.move(leftJoy.magnitude(), leftJoy.reversed().angle());
            return;
        }

        drive.stop();
    }
}
