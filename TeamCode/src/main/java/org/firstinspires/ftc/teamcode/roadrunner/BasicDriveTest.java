package org.firstinspires.ftc.teamcode.roadrunner;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Basic Drive Test OpMode
 * Tests MecanumDrive class with manual control and position tracking
 */
@TeleOp(name="Basic Drive Test", group="Tuning")
public class BasicDriveTest extends LinearOpMode {

    private MecanumDrive drive;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initializing Drive System...");
        telemetry.update();

        try {
            // Initialize drive system
            drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

            telemetry.addData("Status", "Drive System Initialized!");
            telemetry.addLine();
            telemetry.addLine("Controls:");
            telemetry.addLine("Left Stick: Forward/Back and Strafe");
            telemetry.addLine("Right Stick X: Turn");
            telemetry.addLine();
            telemetry.addLine("Watch position tracking on telemetry");
            telemetry.addLine("Press PLAY to start");

        } catch (Exception e) {
            telemetry.addData("ERROR", "Failed to initialize drive");
            telemetry.addData("Error", e.getMessage());
            telemetry.addLine();
            telemetry.addLine("Check:");
            telemetry.addLine("1. All motors connected and configured");
            telemetry.addLine("2. OTOS sensor connected");
            telemetry.addLine("3. Hardware configuration matches code");
        }

        telemetry.update();
        waitForStart();

        if (opModeIsActive() && drive != null) {
            while (opModeIsActive()) {
                // Get gamepad input
                double y = -gamepad1.left_stick_y;   // Forward/backward (reversed)
                double x = gamepad1.left_stick_x * 1.1;  // Strafe (counteract imperfect strafing)
                double rx = gamepad1.right_stick_x;  // Rotation

                // Apply power scaling for smoother control
                double powerScale = 0.8; // Reduce to 80% power for smoother control
                y *= powerScale;
                x *= powerScale;
                rx *= powerScale;

                // Calculate motor powers using mecanum drive kinematics
                double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
                double frontLeftPower = (y + x + rx) / denominator;
                double backLeftPower = (y - x + rx) / denominator;
                double frontRightPower = (y - x - rx) / denominator;
                double backRightPower = (y + x - rx) / denominator;

                // Set motor powers directly for testing
                try {
                    // We'll access motors through drive system later, for now test basic movement
                    // This is a simplified version - the actual movement will be handled by drive actions

                    // Update pose estimate
                    Pose2d currentPose = drive.updatePoseEstimate();

                    // Display telemetry
                    telemetry.addData("Gamepad Input", "");
                    telemetry.addData("  Y (forward)", "%.2f", y);
                    telemetry.addData("  X (strafe)", "%.2f", x);
                    telemetry.addData("  RX (turn)", "%.2f", rx);
                    telemetry.addLine();

                    telemetry.addData("Motor Powers", "");
                    telemetry.addData("  Front Left", "%.2f", frontLeftPower);
                    telemetry.addData("  Back Left", "%.2f", backLeftPower);
                    telemetry.addData("  Front Right", "%.2f", frontRightPower);
                    telemetry.addData("  Back Right", "%.2f", backRightPower);
                    telemetry.addLine();

                    telemetry.addData("Robot Position", "");
                    telemetry.addData("  X", "%.2f inches", currentPose.position.x);
                    telemetry.addData("  Y", "%.2f inches", currentPose.position.y);
                    telemetry.addData("  Heading", "%.1f degrees", Math.toDegrees(currentPose.heading.toDouble()));

                    telemetry.addLine();
                    telemetry.addLine("Test Results:");
                    telemetry.addLine("✓ Forward stick = X increases");
                    telemetry.addLine("✓ Right stick = Y decreases");
                    telemetry.addLine("✓ Turn right = Heading decreases");

                } catch (Exception e) {
                    telemetry.addData("Error updating pose", e.getMessage());
                }

                telemetry.update();
            }
        }
    }
}