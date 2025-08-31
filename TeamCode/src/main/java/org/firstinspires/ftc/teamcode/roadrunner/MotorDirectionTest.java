package org.firstinspires.ftc.teamcode.roadrunner;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Motor Direction Test OpMode
 * Tests each motor individually to verify correct wiring and direction
 */
@TeleOp(name="Motor Direction Test", group="Tuning")
public class MotorDirectionTest extends LinearOpMode {

    private DcMotor leftFront, leftBack, rightBack, rightFront;
    private ElapsedTime timer = new ElapsedTime();

    @Override
    public void runOpMode() {
        // Initialize motors
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");

        // Set initial directions (may need adjustment)
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.FORWARD);

        // Set brake mode
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData("Status", "Initialized");
        telemetry.addLine("This OpMode tests each motor individually");
        telemetry.addLine("Each motor will run for 2 seconds");
        telemetry.addLine("Watch which wheel spins and verify it matches the name");
        telemetry.addLine();
        telemetry.addLine("Press PLAY to start test");
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {
            // Test each motor individually
            testMotor("Left Front", leftFront);
            testMotor("Left Back", leftBack);
            testMotor("Right Back", rightBack);
            testMotor("Right Front", rightFront);

            // Test drive movements
            testDriveMovements();

            telemetry.addData("Status", "Test Complete!");
            telemetry.addLine();
            telemetry.addLine("If any motor spun the wrong wheel:");
            telemetry.addLine("1. Check wiring to correct motor port");
            telemetry.addLine("2. Or flip motor direction in code");
            telemetry.update();
        }
    }

    private void testMotor(String name, DcMotor motor) {
        telemetry.addData("Testing", name);
        telemetry.addLine("Watch the " + name + " wheel - it should spin forward");
        telemetry.update();

        timer.reset();
        while (opModeIsActive() && timer.seconds() < 2.0) {
            motor.setPower(0.5);
        }
        motor.setPower(0);

        sleep(500); // Brief pause between motors
    }

    private void testDriveMovements() {
        telemetry.addLine();
        telemetry.addData("Testing", "Basic Drive Movements");
        telemetry.update();

        // Test forward movement
        testMovement("Forward", 0.5, 0.5, 0.5, 0.5);

        // Test backward movement
        testMovement("Backward", -0.5, -0.5, -0.5, -0.5);

        // Test strafe right
        testMovement("Strafe Right", 0.5, -0.5, 0.5, -0.5);

        // Test strafe left
        testMovement("Strafe Left", -0.5, 0.5, -0.5, 0.5);

        // Test turn right
        testMovement("Turn Right", 0.5, 0.5, -0.5, -0.5);

        // Test turn left
        testMovement("Turn Left", -0.5, -0.5, 0.5, 0.5);
    }

    private void testMovement(String name, double lf, double lb, double rf, double rb) {
        telemetry.addData("Testing", name);
        telemetry.addLine("Robot should move: " + name);
        telemetry.update();

        timer.reset();
        while (opModeIsActive() && timer.seconds() < 1.5) {
            leftFront.setPower(lf);
            leftBack.setPower(lb);
            rightFront.setPower(rf);
            rightBack.setPower(rb);
        }

        // Stop all motors
        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);

        sleep(500); // Brief pause between movements
    }
}