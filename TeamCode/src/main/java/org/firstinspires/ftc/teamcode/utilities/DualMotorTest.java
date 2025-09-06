package org.firstinspires.ftc.teamcode.utilities;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;


@TeleOp(name = "MotorTest")
public class DualMotorTest extends LinearOpMode {

    DcMotor leftMotor;
    DcMotor rightMotor;

    @Override
    public void runOpMode() throws InterruptedException {
        leftMotor = hardwareMap.dcMotor.get("left_motor");
        rightMotor = hardwareMap.dcMotor.get("right_motor");

        leftMotor.setDirection(DcMotor.Direction.REVERSE);

        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftMotor.setTargetPosition(1000);
        rightMotor.setTargetPosition(1000);

        // Power sign matters again as we are running without encoder.
        leftMotor.setPower(1);
        rightMotor.setPower(1);

        //sleep(2000);

        while (opModeIsActive() && leftMotor.getCurrentPosition() < leftMotor.getTargetPosition())
        {
            telemetry.addData("encoder-back-left", leftMotor.getCurrentPosition());
            telemetry.addData("encoder-back-right", rightMotor.getCurrentPosition());
            telemetry.update();
            idle();
        }

        leftMotor.setPower(0);
        rightMotor.setPower(0);
    }
}