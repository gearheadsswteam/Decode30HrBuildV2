package org.firstinspires.ftc.teamcode.roadrunner;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Drive Tuning Test OpMode
 * Tests Roadrunner actions for autonomous movement
 */
@TeleOp(name="Drive Tuning Test", group="Tuning")
public class DriveTuningTest extends LinearOpMode {

    private MecanumDrive drive;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initializing...");
        telemetry.update();

        try {
            // Initialize drive system
            drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

            telemetry.addData("Status", "Ready for Drive Testing!");
            telemetry.addLine();
            telemetry.addLine("This OpMode tests autonomous movements:");
            telemetry.addLine();
            telemetry.addLine("A - Drive forward 24 inches");
            telemetry.addLine("B - Strafe right 24 inches");
            telemetry.addLine("X - Turn 90 degrees left");
            telemetry.addLine("Y - Drive to point (24, 24)");
            telemetry.addLine();
            telemetry.addLine("START - Reset position to (0, 0, 0)");
            telemetry.addLine();
            telemetry.addLine("Watch robot movement and check accuracy!");
            telemetry.addLine("Press PLAY to start");

        } catch (Exception e) {
            telemetry.addData("ERROR", "Failed to initialize");
            telemetry.addData("Error", e.getMessage());
        }

        telemetry.update();
        waitForStart();

        if (opModeIsActive() && drive != null) {
            while (opModeIsActive()) {
                // Reset position
                if (gamepad1.start) {
                    drive.setPose(new Pose2d(0, 0, 0));
                    telemetry.addData("Status", "Position Reset!");
                    telemetry.update();
                    sleep(500);
                }

                // Test movements
                if (gamepad1.a) {
                    testForwardDrive();
                }

                if (gamepad1.b) {
                    testStrafeRight();
                }

                if (gamepad1.x) {
                    testTurnLeft();
                }

                if (gamepad1.y) {
                    testDriveToPoint();
                }

                // Update and display current position
                Pose2d currentPose = drive.updatePoseEstimate();

                telemetry.addData("Current Position", "");
                telemetry.addData("  X", "%.2f inches", currentPose.position.x);
                telemetry.addData("  Y", "%.2f inches", currentPose.position.y);
                telemetry.addData("  Heading", "%.1f degrees", Math.toDegrees(currentPose.heading.toDouble()));
                telemetry.addLine();

                telemetry.addLine("CONTROLS:");
                telemetry.addData("A", "Drive Forward 24\"");
                telemetry.addData("B", "Strafe Right 24\"");
                telemetry.addData("X", "Turn Left 90°");
                telemetry.addData("Y", "Drive to (24, 24)");
                telemetry.addData("START", "Reset Position");
                telemetry.addLine();

                telemetry.addLine("Check movement accuracy!");
                telemetry.addLine("Adjust drive constants if needed");

                telemetry.update();
            }
        }
    }

    private void testForwardDrive() {
        telemetry.addData("Test", "Driving Forward 24 inches...");
        telemetry.update();

        // Create action to drive forward 24 inches
        Action driveAction = drive.strafeToLinearHeading(
                new Vector2d(24, 0),
                0  // Keep heading at 0 degrees
        );

        // Execute action
        executeAction(driveAction, "Forward Drive Test");
    }

    private void testStrafeRight() {
        telemetry.addData("Test", "Strafing Right 24 inches...");
        telemetry.update();

        Pose2d currentPose = drive.updatePoseEstimate();

        // Create action to strafe right 24 inches from current position
        Action strafeAction = drive.strafeToLinearHeading(
                new Vector2d(currentPose.position.x, currentPose.position.y - 24),
                currentPose.heading.toDouble()
        );

        // Execute action
        executeAction(strafeAction, "Strafe Right Test");
    }

    private void testTurnLeft() {
        telemetry.addData("Test", "Turning Left 90 degrees...");
        telemetry.update();

        Pose2d currentPose = drive.updatePoseEstimate();

        // Create action to turn left 90 degrees
        Action turnAction = drive.turnTo(currentPose.heading.toDouble() + Math.toRadians(90));

        // Execute action
        executeAction(turnAction, "Turn Left Test");
    }

    private void testDriveToPoint() {
        telemetry.addData("Test", "Driving to point (24, 24)...");
        telemetry.update();

        // Create action to drive to specific point
        Action driveAction = drive.strafeToLinearHeading(
                new Vector2d(24, 24),
                Math.toRadians(45)  // Face 45 degrees at destination
        );

        // Execute action
        executeAction(driveAction, "Drive to Point Test");
    }

    private void executeAction(Action action, String testName) {
        TelemetryPacket packet = new TelemetryPacket();

        // Run the action until completion
        while (opModeIsActive() && action.run(packet)) {
            // Update pose estimate
            Pose2d currentPose = drive.updatePoseEstimate();

            // Display progress
            telemetry.addData("Test", testName);
            telemetry.addData("Status", "In Progress...");
            telemetry.addLine();
            telemetry.addData("Current Position", "");
            telemetry.addData("  X", "%.2f inches", currentPose.position.x);
            telemetry.addData("  Y", "%.2f inches", currentPose.position.y);
            telemetry.addData("  Heading", "%.1f degrees", Math.toDegrees(currentPose.heading.toDouble()));

            // Show any telemetry from the action
            if (packet.fieldOverlay().getOperations().size() > 0) {
                telemetry.addLine();
                telemetry.addLine("Action Telemetry:");
                // Add packet data if available
            }

            telemetry.update();

            sleep(20); // Small delay for telemetry update
        }

        // Action completed
        Pose2d finalPose = drive.updatePoseEstimate();

        telemetry.addData("Test", testName);
        telemetry.addData("Status", "COMPLETED!");
        telemetry.addLine();
        telemetry.addData("Final Position", "");
        telemetry.addData("  X", "%.2f inches", finalPose.position.x);
        telemetry.addData("  Y", "%.2f inches", finalPose.position.y);
        telemetry.addData("  Heading", "%.1f degrees", Math.toDegrees(finalPose.heading.toDouble()));
        telemetry.addLine();
        telemetry.addLine("Measure actual position and compare!");
        telemetry.update();

        sleep(2000); // Show results for 2 seconds
    }
}