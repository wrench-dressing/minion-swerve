package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "Reset Encoders")
public class EncoderResetTeleOp extends LinearOpMode {
    private void resetEncoder(DcMotorEx motor) {
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void runOpMode() {
        telemetry.addData("Info", "Start to reset encoders");
        telemetry.update();
        waitForStart();

        resetEncoder(hardwareMap.get(DcMotorEx.class, Configuration.FRONT_TOP_MOTOR));
        resetEncoder(hardwareMap.get(DcMotorEx.class, Configuration.FRONT_BOTTOM_MOTOR));
        resetEncoder(hardwareMap.get(DcMotorEx.class, Configuration.BACK_TOP_MOTOR));
        resetEncoder(hardwareMap.get(DcMotorEx.class, Configuration.BACK_BOTTOM_MOTOR));

        while (opModeIsActive()) {
            telemetry.addData("Info", "Reset encoders!");
            telemetry.update();
        }
    }
}
