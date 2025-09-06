package org.firstinspires.ftc.teamcode.utilities;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

public class SingleMotorStepTest extends LinearOpMode {
    DcMotor testMotor;
    int currentTarget = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        // Map the motor (make sure the name matches your RC configuration)
        testMotor = hardwareMap.dcMotor.get("test_motor");

        // Set direction (reverse if needed)
        testMotor.setDirection(DcMotor.Direction.FORWARD);

        // Reset encoder and set mode
        testMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        testMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        telemetry.addLine("Ready to run motor step test. A=+10, B=+50, X=-10, Y=-50");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // --- Step +10 ---
            if (gamepad1.a) {
                currentTarget += 10;
                moveToTarget(currentTarget);
            }

            // --- Step +50 ---
            if (gamepad1.b) {
                currentTarget += 50;
                moveToTarget(currentTarget);
            }

            // --- Step -10 ---
            if (gamepad1.x) {
                currentTarget -= 10;
                moveToTarget(currentTarget);
            }

            // --- Step -50 ---
            if (gamepad1.y) {
                currentTarget -= 50;
                moveToTarget(currentTarget);
            }

            telemetry.addData("Motor Encoder", testMotor.getCurrentPosition());
            telemetry.addData("Target", currentTarget);
            telemetry.update();
        }

        // Stop motor at the end
        testMotor.setPower(0);
    }

    /** Helper to move motor to given encoder target */
    private void moveToTarget(int target) {
        testMotor.setTargetPosition(target);
        testMotor.setPower(0.5);

        while (opModeIsActive() && testMotor.isBusy()) {
            telemetry.addData("Motor Encoder", testMotor.getCurrentPosition());
            telemetry.addData("Target", target);
            telemetry.update();
            idle();
        }

        testMotor.setPower(0);
    }
}
