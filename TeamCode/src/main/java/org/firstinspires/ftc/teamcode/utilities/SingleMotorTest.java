package org.firstinspires.ftc.teamcode.utilities;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Single Motor Test", group = "Test")
public class SingleMotorTest extends LinearOpMode {

    DcMotor testMotor;

    @Override
    public void runOpMode() throws InterruptedException {
        // Map the motor (make sure the name matches your RC configuration)
        testMotor = hardwareMap.dcMotor.get("test_motor");

        // Set direction (reverse if motor spins the wrong way)
        testMotor.setDirection(DcMotor.Direction.FORWARD);

        // Reset encoder and use RUN_TO_POSITION
        testMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        testMotor.setTargetPosition(1000);
        testMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        waitForStart();

        // Set power and let it run to target
        testMotor.setPower(1.0);

        while (opModeIsActive() && testMotor.isBusy()) {
            telemetry.addData("Motor Encoder", testMotor.getCurrentPosition());
            telemetry.addData("Target", testMotor.getTargetPosition());
            telemetry.update();
            idle();
        }

        // Stop motor
        testMotor.setPower(0);
    }
}
