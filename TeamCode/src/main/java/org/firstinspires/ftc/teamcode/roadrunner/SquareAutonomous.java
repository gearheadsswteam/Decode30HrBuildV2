package org.firstinspires.ftc.teamcode.roadrunner;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Square Autonomous OpMode
 * Drives robot in a 24-inch square pattern for 4 complete rotations
 * Uses Roadrunner 1.0.1 with SparkFun OTOS localization
 */
@Autonomous(name="Square Autonomous", group="Final")
public class SquareAutonomous extends LinearOpMode {

    private MecanumDrive drive;
    private ElapsedTime runtime = new ElapsedTime();

    // Square parameters
    private final double SQUARE_SIZE = 24.0;  // inches
    private final int NUM_ROTATIONS = 4;      // number of complete squares

    // Square corner positions (starting at origin, going clockwise)
    private final Vector2d[] SQUARE_CORNERS = {
            new Vector2d(0, 0),           // Start position (corner 1)
            new Vector2d(SQUARE_SIZE, 0), // Corner 2 (right)
            new Vector2d(SQUARE_SIZE, SQUARE_SIZE), // Corner 3 (top-right)
            new Vector2d(0, SQUARE_SIZE), // Corner 4 (top-left)
            new Vector2d(0, 0)            // Back to start (corner 1)
    };

    // Headings for each corner (facing direction of next movement)
    private final double[] CORNER_HEADINGS = {
            Math.toRadians(0),    // Face right (0°)
            Math.toRadians(90),   // Face up (90°)
            Math.toRadians(180),  // Face left (180°)
            Math.toRadians(270),  // Face down (270°)
            Math.toRadians(0)     // Face right again (0°)
    };

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initializing Square Autonomous...");
        telemetry.update();

        try {
            // Initialize drive system at starting position
            drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

            telemetry.addData("Status", "✓ Drive System Initialized");
            telemetry.addData("Square Size", "%.0f inches", SQUARE_SIZE);
            telemetry.addData("Number of Rotations", "%d", NUM_ROTATIONS);
            telemetry.addData("Total Distance", "%.0f inches", SQUARE_SIZE * 4 * NUM_ROTATIONS);
            telemetry.addLine();
            telemetry.addLine("ROBOT SETUP:");
            telemetry.addLine("1. Place robot at starting position");
            telemetry.addLine("2. Robot should face RIGHT (0° heading)");
            telemetry.addLine("3. Ensure 24\"+ clear space in all directions");
            telemetry.addLine();
            telemetry.addLine("SQUARE PATH (clockwise):");
            telemetry.addLine("Start(0,0) → Right(24,0) → Up(24,24) → Left(0,24) → Start(0,0)");
            telemetry.addLine();
            telemetry.addLine("Ready to run! Press PLAY to start.");

        } catch (Exception e) {
            telemetry.addData("ERROR", "Failed to initialize");
            telemetry.addData("Details", e.getMessage());
            telemetry.addLine();
            telemetry.addLine("Check:");
            telemetry.addLine("• All motors connected and configured");
            telemetry.addLine("• OTOS sensor connected to I2C");
            telemetry.addLine("• Hardware configuration matches code");
        }

        telemetry.update();
        waitForStart();

        if (opModeIsActive() && drive != null) {
            runtime.reset();

            telemetry.addData("Status", "🚀 STARTING SQUARE AUTONOMOUS!");
            telemetry.update();

            // Execute the square pattern for specified number of rotations
            for (int rotation = 1; rotation <= NUM_ROTATIONS && opModeIsActive(); rotation++) {
                telemetry.addData("Status", String.format("🔄 Starting Rotation %d of %d", rotation, NUM_ROTATIONS));
                telemetry.update();

                // Drive the square pattern
                driveSquarePattern(rotation);

                if (opModeIsActive()) {
                    telemetry.addData("Status", String.format("✓ Completed Rotation %d", rotation));
                    telemetry.update();
                    sleep(500); // Brief pause between rotations
                }
            }

            // Final status
            if (opModeIsActive()) {
                Pose2d finalPose = drive.updatePoseEstimate();

                telemetry.clear();
                telemetry.addData("🏁 STATUS", "AUTONOMOUS COMPLETE!");
                telemetry.addLine();
                telemetry.addData("Total Runtime", "%.1f seconds", runtime.seconds());
                telemetry.addData("Rotations Completed", "%d of %d", NUM_ROTATIONS, NUM_ROTATIONS);
                telemetry.addData("Total Distance", "%.0f inches", SQUARE_SIZE * 4 * NUM_ROTATIONS);
                telemetry.addLine();
                telemetry.addData("Final Position", "");
                telemetry.addData("  X", "%.2f inches (target: 0.00)", finalPose.position.x);
                telemetry.addData("  Y", "%.2f inches (target: 0.00)", finalPose.position.y);
                telemetry.addData("  Heading", "%.1f° (target: 0.0°)", Math.toDegrees(finalPose.heading.toDouble()));
                telemetry.addLine();

                // Calculate accuracy
                double positionError = Math.sqrt(
                        finalPose.position.x * finalPose.position.x +
                                finalPose.position.y * finalPose.position.y
                );
                double headingError = Math.abs(Math.toDegrees(finalPose.heading.toDouble()));

                telemetry.addData("Position Accuracy", "%.2f inches from start", positionError);
                telemetry.addData("Heading Accuracy", "%.1f° from start", headingError);
                telemetry.addLine();

                if (positionError < 2.0 && headingError < 10.0) {
                    telemetry.addLine("🎯 EXCELLENT ACCURACY!");
                } else if (positionError < 4.0 && headingError < 20.0) {
                    telemetry.addLine("✅ GOOD ACCURACY!");
                } else {
                    telemetry.addLine("⚠️ Consider tuning calibration");
                }

                telemetry.update();
            }
        }
    }

    /**
     * Drive one complete square pattern (4 sides)
     */
    private void driveSquarePattern(int rotationNumber) {
        for (int corner = 1; corner < SQUARE_CORNERS.length && opModeIsActive(); corner++) {
            Vector2d targetPosition = SQUARE_CORNERS[corner];
            double targetHeading = CORNER_HEADINGS[corner];

            // Create action to drive to next corner
            Action moveToCorner = drive.strafeToLinearHeading(targetPosition, targetHeading);

            // Execute the movement
            executeMoveToCorner(moveToCorner, rotationNumber, corner, targetPosition, targetHeading);

            // Brief pause at each corner
            if (opModeIsActive()) {
                sleep(200);
            }
        }
    }

    /**
     * Execute movement to a specific corner with telemetry
     */
    private void executeMoveToCorner(Action action, int rotation, int corner, Vector2d target, double targetHeading) {
        String[] cornerNames = {"Start", "Right", "Top-Right", "Top-Left", "Start"};
        TelemetryPacket packet = new TelemetryPacket();

        telemetry.addData("Rotation", "%d of %d", rotation, NUM_ROTATIONS);
        telemetry.addData("Moving to", "%s Corner", cornerNames[corner]);
        telemetry.addData("Target", "(%.0f, %.0f) @ %.0f°",
                target.x, target.y, Math.toDegrees(targetHeading));
        telemetry.update();

        // Execute action with real-time feedback
        while (opModeIsActive() && action.run(packet)) {
            Pose2d currentPose = drive.updatePoseEstimate();

            // Calculate progress
            double distanceToTarget = Math.sqrt(
                    Math.pow(target.x - currentPose.position.x, 2) +
                            Math.pow(target.y - currentPose.position.y, 2)
            );

            double headingError = Math.abs(targetHeading - currentPose.heading.toDouble());
            if (headingError > Math.PI) headingError = 2 * Math.PI - headingError;

            // Update telemetry
            telemetry.addData("🔄 Rotation", "%d of %d", rotation, NUM_ROTATIONS);
            telemetry.addData("➡️ Moving to", "%s Corner", cornerNames[corner]);
            telemetry.addLine();
            telemetry.addData("Current Position", "");
            telemetry.addData("  X", "%.2f inches", currentPose.position.x);
            telemetry.addData("  Y", "%.2f inches", currentPose.position.y);
            telemetry.addData("  Heading", "%.1f degrees", Math.toDegrees(currentPose.heading.toDouble()));
            telemetry.addLine();
            telemetry.addData("Target", "(%.0f, %.0f) @ %.0f°",
                    target.x, target.y, Math.toDegrees(targetHeading));
            telemetry.addData("Distance to Target", "%.2f inches", distanceToTarget);
            telemetry.addData("Heading Error", "%.1f degrees", Math.toDegrees(headingError));
            telemetry.addLine();
            telemetry.addData("Runtime", "%.1f seconds", runtime.seconds());

            telemetry.update();

            sleep(50); // Update rate control
        }

        // Arrival confirmation
        Pose2d finalPose = drive.updatePoseEstimate();
        telemetry.addData("✅ Arrived at", "%s Corner", cornerNames[corner]);
        telemetry.addData("Position", "(%.2f, %.2f) @ %.1f°",
                finalPose.position.x, finalPose.position.y, Math.toDegrees(finalPose.heading.toDouble()));
        telemetry.update();
    }
}