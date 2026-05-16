package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@SuppressWarnings("unused")
@TeleOp(name = "Minion Swerve TeleOp")
public class MinionSwerveTeleOp extends LinearOpMode {
    public static Telemetry g_tele;
    Drivetrain drive;
    IMU imu;

    @Override
    public void runOpMode() {
        waitForStart();

        g_tele = telemetry;
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.RIGHT, RevHubOrientationOnRobot.UsbFacingDirection.UP)));
        imu.resetYaw();
        drive = new Drivetrain(hardwareMap, imu);

        while (opModeIsActive()) {
            telemetry.addData("Instructions", "Left joy for translation." +
                    "Triggers to turn. B = toggle Fast Mode." +
                    "Options = Reset encoders. Start = Reset IMU.");

            if (gamepad1.startWasPressed())
                imu.resetYaw();

            movement();

            telemetry.update();
        }
    }

    private void movement() {
        Vector leftJoy = new Vector(gamepad1.left_stick_x, gamepad1.left_stick_y - 0.01).deadbanded(0.1);
        drive.move(leftJoy.reversed(), gamepad1.right_stick_x * Configuration.angularMaxSpeed);
    }
}
