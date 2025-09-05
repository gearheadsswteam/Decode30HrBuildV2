package org.firstinspires.ftc.teamcode.roadrunner.rrtest;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive; // Adjust import to match your path

@Config
@TeleOp(name = "Dashboard PID Tuning", group = "Tuning")
public class DashboardPIDTuning extends LinearOpMode {

    // PID GAINS - These will appear on FTC Dashboard for real-time tuning
    public static double AXIAL_GAIN = 0.5;         // Forward/backward P gain
    public static double LATERAL_GAIN = 0.5;       // Left/right P gain
    public static double HEADING_GAIN = 0.5;       // Rotation P gain

    public static double AXIAL_VEL_GAIN = 0.0;     // Forward/backward velocity gain
    public static double LATERAL_VEL_GAIN = 0.0;   // Left/right velocity gain
    public static double HEADING_VEL_GAIN = 0.0;   // Rotation velocity gain

    // TEST PARAMETERS - Adjustable on Dashboard
    public static double STRAIGHT_DISTANCE = 24.0;  // Distance for straight test (inches)
    public static double STRAFE_DISTANCE = 24.0;    // Distance for strafe test (inches)
    public static double TURN_ANGLE = 90.0;         // Angle for turn test (degrees)

    // DRIVE CONSTRAINTS - Adjustable on Dashboard
    public static double MAX_VEL = 30.0;            // Max velocity (inches/sec)
    public static double MAX_ACCEL = 50.0;          // Max acceleration (inches/sec²)
    public static double MAX_ANG_VEL = 180.0;       // Max angular velocity (degrees/sec)
    public static double MAX_ANG_ACCEL = 180.0;     // Max angular acceleration (degrees/sec²)

    // CONTROL SENSITIVITY - For manual driving feel
    public static double DRIVE_SENSITIVITY = 0.8;   // Manual drive power multiplier

    private MecanumDrive drive;
    private FtcDashboard dashboard;
    private boolean testRunning = false;

    @Override
    public void runOpMode() {
        // Initialize drive system
        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

        // Setup Dashboard telemetry
        dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        telemetry.addLine("=== DASHBOARD PID TUNING ===");
        telemetry.addLine("Open FTC Dashboard in browser:");
        telemetry.addLine("http://192.168.43.1:8080/dash");
        telemetry.addLine();
        telemetry.addLine("Adjust PID values in Configuration tab");
        telemetry.addLine();
        telemetry.addLine("Controls:");
        telemetry.addLine("A: Straight line test");
        telemetry.addLine("B: Strafe test");
        telemetry.addLine("Y: Turn test");
        telemetry.addLine("X: Reset pose to origin");
        telemetry.addLine("Left stick: Manual drive (feel test)");
        telemetry.addLine("Right stick: Manual rotation");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Update PID gains from Dashboard values
            updatePIDGains();

            // Handle controls
            handleGamepadInputs();

            // Update telemetry
            updateTelemetry();

            // Small delay to prevent excessive updates
            sleep(50);
        }
    }

    private void updatePIDGains() {
        // Update HolonomicController with Dashboard values
        try {
            drive.updatePIDGains(AXIAL_GAIN, LATERAL_GAIN, HEADING_GAIN,
                    AXIAL_VEL_GAIN, LATERAL_VEL_GAIN, HEADING_VEL_GAIN);

            drive.updateConstraints(MAX_VEL, MAX_ACCEL,
                    Math.toRadians(MAX_ANG_VEL), Math.toRadians(MAX_ANG_ACCEL));
        } catch (Exception e) {
            telemetry.addLine("Error updating PID gains - check MecanumDrive methods");
        }
    }

    private void handleGamepadInputs() {
        if (!testRunning) {
            if (gamepad1.a) {
                runStraightLineTest();
            } else if (gamepad1.b) {
                runStrafeTest();
            } else if (gamepad1.y) {
                runTurnTest();
            } else if (gamepad1.x) {
                resetPose();
            } else {
                manualDrive();
            }
        }
    }

    private void manualDrive() {
        // Manual driving for "feel" testing of PID responsiveness
        double x = -gamepad1.left_stick_y * DRIVE_SENSITIVITY;   // Forward/back
        double y = gamepad1.left_stick_x * DRIVE_SENSITIVITY;    // Strafe left/right
        double rx = gamepad1.right_stick_x * DRIVE_SENSITIVITY;  // Rotate

        // Apply deadzone
        if (Math.abs(x) < 0.1) x = 0;
        if (Math.abs(y) < 0.1) y = 0;
        if (Math.abs(rx) < 0.1) rx = 0;

        drive.setDrivePowers(new PoseVelocity2d(new Vector2d(x, y), rx));
        drive.updatePoseEstimate();
    }

    private void resetPose() {
        drive.setPose(new Pose2d(0, 0, 0));
        telemetry.addLine("Pose reset to (0, 0, 0°)");
        telemetry.update();
        sleep(500);
    }

    private void runStraightLineTest() {
        testRunning = true;
        telemetry.addLine("=== RUNNING STRAIGHT LINE TEST ===");
        telemetry.addData("Target Distance", STRAIGHT_DISTANCE + " inches");
        telemetry.update();

        Pose2d startPose = drive.getPose();

        Action trajectory = drive.actionBuilder(startPose)
                .lineToX(startPose.position.x + STRAIGHT_DISTANCE)
                .build();

        long startTime = System.currentTimeMillis();
        double maxYDrift = 0;

        while (opModeIsActive() && trajectory.run(new TelemetryPacket())) {
            drive.updatePoseEstimate();
            Pose2d currentPose = drive.getPose();

            double progress = currentPose.position.x - startPose.position.x;
            double yDrift = Math.abs(currentPose.position.y - startPose.position.y);
            double headingDrift = Math.abs(Math.toDegrees(currentPose.heading.toDouble() - startPose.heading.toDouble()));
            double elapsed = (System.currentTimeMillis() - startTime) / 1000.0;

            maxYDrift = Math.max(maxYDrift, yDrift);

            telemetry.addLine("=== STRAIGHT LINE TEST RUNNING ===");
            telemetry.addData("Target", STRAIGHT_DISTANCE + " inches");
            telemetry.addData("Progress", "%.2f inches", progress);
            telemetry.addData("Y Drift", "%.2f inches", yDrift);
            telemetry.addData("Heading Drift", "%.1f degrees", headingDrift);
            telemetry.addData("Time", "%.1f seconds", elapsed);
            telemetry.addLine();
            displayCurrentPIDGains();
            telemetry.update();
        }

        analyzeStraightLineResults(startPose, maxYDrift);
        testRunning = false;
    }

    private void runStrafeTest() {
        testRunning = true;
        telemetry.addLine("=== RUNNING STRAFE TEST ===");
        telemetry.addData("Target Distance", STRAFE_DISTANCE + " inches left");
        telemetry.update();

        Pose2d startPose = drive.getPose();

        Action trajectory = drive.actionBuilder(startPose)
                .lineToY(startPose.position.y + STRAFE_DISTANCE)
                .build();

        long startTime = System.currentTimeMillis();
        double maxXDrift = 0;

        while (opModeIsActive() && trajectory.run(new TelemetryPacket())) {
            drive.updatePoseEstimate();
            Pose2d currentPose = drive.getPose();

            double progress = currentPose.position.y - startPose.position.y;
            double xDrift = Math.abs(currentPose.position.x - startPose.position.x);
            double headingDrift = Math.abs(Math.toDegrees(currentPose.heading.toDouble() - startPose.heading.toDouble()));
            double elapsed = (System.currentTimeMillis() - startTime) / 1000.0;

            maxXDrift = Math.max(maxXDrift, xDrift);

            telemetry.addLine("=== STRAFE TEST RUNNING ===");
            telemetry.addData("Target", STRAFE_DISTANCE + " inches");
            telemetry.addData("Progress", "%.2f inches", progress);
            telemetry.addData("X Drift", "%.2f inches", xDrift);
            telemetry.addData("Heading Drift", "%.1f degrees", headingDrift);
            telemetry.addData("Time", "%.1f seconds", elapsed);
            telemetry.addLine();
            displayCurrentPIDGains();
            telemetry.update();
        }

        analyzeStrafeResults(startPose, maxXDrift);
        testRunning = false;
    }

    private void runTurnTest() {
        testRunning = true;
        telemetry.addLine("=== RUNNING TURN TEST ===");
        telemetry.addData("Target Angle", TURN_ANGLE + " degrees");
        telemetry.update();

        Pose2d startPose = drive.getPose();

        Action trajectory = drive.actionBuilder(startPose)
                .turn(Math.toRadians(TURN_ANGLE))
                .build();

        long startTime = System.currentTimeMillis();
        double maxPositionDrift = 0;

        while (opModeIsActive() && trajectory.run(new TelemetryPacket())) {
            drive.updatePoseEstimate();
            Pose2d currentPose = drive.getPose();

            double currentAngle = Math.toDegrees(currentPose.heading.toDouble() - startPose.heading.toDouble());
            double positionDrift = Math.sqrt(
                    Math.pow(currentPose.position.x - startPose.position.x, 2) +
                            Math.pow(currentPose.position.y - startPose.position.y, 2)
            );
            double elapsed = (System.currentTimeMillis() - startTime) / 1000.0;

            maxPositionDrift = Math.max(maxPositionDrift, positionDrift);

            telemetry.addLine("=== TURN TEST RUNNING ===");
            telemetry.addData("Target", TURN_ANGLE + " degrees");
            telemetry.addData("Current Angle", "%.1f degrees", currentAngle);
            telemetry.addData("Position Drift", "%.2f inches", positionDrift);
            telemetry.addData("Max Drift", "%.2f inches", maxPositionDrift);
            telemetry.addData("Time", "%.1f seconds", elapsed);
            telemetry.addLine();
            displayCurrentPIDGains();
            telemetry.update();
        }

        analyzeTurnResults(startPose, maxPositionDrift);
        testRunning = false;
    }

    private void analyzeStraightLineResults(Pose2d startPose, double maxYDrift) {
        drive.updatePoseEstimate();
        Pose2d finalPose = drive.getPose();

        double distanceError = Math.abs(STRAIGHT_DISTANCE - (finalPose.position.x - startPose.position.x));
        double finalYDrift = Math.abs(finalPose.position.y - startPose.position.y);
        double headingDrift = Math.abs(Math.toDegrees(finalPose.heading.toDouble() - startPose.heading.toDouble()));

        telemetry.addLine("=== STRAIGHT LINE RESULTS ===");
        telemetry.addData("Distance Error", "%.2f inches", distanceError);
        telemetry.addData("Final Y Drift", "%.2f inches", finalYDrift);
        telemetry.addData("Max Y Drift", "%.2f inches", maxYDrift);
        telemetry.addData("Heading Drift", "%.1f degrees", headingDrift);
        telemetry.addLine();

        // Provide tuning guidance
        if (distanceError > 2.0) {
            telemetry.addLine("Distance Error > 2\":");
            telemetry.addLine("- Try adjusting AXIAL_GAIN");
            if (distanceError > 5.0) {
                telemetry.addLine("- Large error - increase AXIAL_GAIN");
            }
        }
        if (maxYDrift > 1.0) {
            telemetry.addLine("Y Drift > 1\":");
            telemetry.addLine("- Check wheel alignment");
            telemetry.addLine("- Try adjusting LATERAL_GAIN");
        }
        if (headingDrift > 5.0) {
            telemetry.addLine("Heading Drift > 5°:");
            telemetry.addLine("- Try adjusting HEADING_GAIN");
        }

        // Overall assessment
        if (distanceError < 1.0 && maxYDrift < 0.5 && headingDrift < 3.0) {
            telemetry.addLine("EXCELLENT straight line performance!");
        } else if (distanceError < 2.0 && maxYDrift < 1.0 && headingDrift < 5.0) {
            telemetry.addLine("GOOD straight line performance");
        } else {
            telemetry.addLine("Needs more tuning");
        }

        telemetry.update();
        sleep(4000);
    }

    private void analyzeStrafeResults(Pose2d startPose, double maxXDrift) {
        drive.updatePoseEstimate();
        Pose2d finalPose = drive.getPose();

        double distanceError = Math.abs(STRAFE_DISTANCE - (finalPose.position.y - startPose.position.y));
        double finalXDrift = Math.abs(finalPose.position.x - startPose.position.x);
        double headingDrift = Math.abs(Math.toDegrees(finalPose.heading.toDouble() - startPose.heading.toDouble()));

        telemetry.addLine("=== STRAFE RESULTS ===");
        telemetry.addData("Distance Error", "%.2f inches", distanceError);
        telemetry.addData("Final X Drift", "%.2f inches", finalXDrift);
        telemetry.addData("Max X Drift", "%.2f inches", maxXDrift);
        telemetry.addData("Heading Drift", "%.1f degrees", headingDrift);
        telemetry.addLine();

        // Provide tuning guidance
        if (distanceError > 2.0) {
            telemetry.addLine("Distance Error > 2\":");
            telemetry.addLine("- Try adjusting LATERAL_GAIN");
        }
        if (maxXDrift > 1.0) {
            telemetry.addLine("X Drift > 1\":");
            telemetry.addLine("- Check mecanum wheel condition");
            telemetry.addLine("- Try adjusting AXIAL_GAIN");
        }
        if (headingDrift > 5.0) {
            telemetry.addLine("Heading Drift > 5°:");
            telemetry.addLine("- Try adjusting HEADING_GAIN");
        }

        // Overall assessment
        if (distanceError < 1.0 && maxXDrift < 0.5 && headingDrift < 3.0) {
            telemetry.addLine("EXCELLENT strafe performance!");
        } else if (distanceError < 2.0 && maxXDrift < 1.0 && headingDrift < 5.0) {
            telemetry.addLine("GOOD strafe performance");
        } else {
            telemetry.addLine("Needs more tuning");
        }

        telemetry.update();
        sleep(4000);
    }

    private void analyzeTurnResults(Pose2d startPose, double maxPositionDrift) {
        drive.updatePoseEstimate();
        Pose2d finalPose = drive.getPose();

        double headingError = Math.abs(TURN_ANGLE - Math.toDegrees(finalPose.heading.toDouble() - startPose.heading.toDouble()));
        double finalPositionDrift = Math.sqrt(
                Math.pow(finalPose.position.x - startPose.position.x, 2) +
                        Math.pow(finalPose.position.y - startPose.position.y, 2)
        );

        telemetry.addLine("=== TURN RESULTS ===");
        telemetry.addData("Heading Error", "%.1f degrees", headingError);
        telemetry.addData("Final Position Drift", "%.2f inches", finalPositionDrift);
        telemetry.addData("Max Position Drift", "%.2f inches", maxPositionDrift);
        telemetry.addLine();

        // Provide tuning guidance
        if (headingError > 5.0) {
            telemetry.addLine("Heading Error > 5°:");
            if (headingError > 15.0) {
                telemetry.addLine("- Large error - increase HEADING_GAIN");
            } else {
                telemetry.addLine("- Try fine-tuning HEADING_GAIN");
            }
        }
        if (maxPositionDrift > 2.0) {
            telemetry.addLine("Position Drift > 2\":");
            telemetry.addLine("- Check AXIAL_GAIN and LATERAL_GAIN");
            telemetry.addLine("- Ensure drive base is square");
        }

        // Overall assessment
        if (headingError < 3.0 && maxPositionDrift < 1.0) {
            telemetry.addLine("EXCELLENT turn performance!");
        } else if (headingError < 5.0 && maxPositionDrift < 2.0) {
            telemetry.addLine("GOOD turn performance");
        } else {
            telemetry.addLine("Needs more tuning");
        }

        telemetry.update();
        sleep(4000);
    }

    private void displayCurrentPIDGains() {
        telemetry.addLine("Current PID Gains:");
        telemetry.addData("Axial", AXIAL_GAIN);
        telemetry.addData("Lateral", LATERAL_GAIN);
        telemetry.addData("Heading", HEADING_GAIN);
        if (AXIAL_VEL_GAIN != 0 || LATERAL_VEL_GAIN != 0 || HEADING_VEL_GAIN != 0) {
            telemetry.addData("Axial Vel", AXIAL_VEL_GAIN);
            telemetry.addData("Lateral Vel", LATERAL_VEL_GAIN);
            telemetry.addData("Heading Vel", HEADING_VEL_GAIN);
        }
    }

    private void updateTelemetry() {
        if (!testRunning) {
            drive.updatePoseEstimate();
            Pose2d pose = drive.getPose();

            telemetry.addLine("=== DASHBOARD PID TUNING ===");
            telemetry.addData("Current Position", "%.1f, %.1f, %.1f°",
                    pose.position.x, pose.position.y, Math.toDegrees(pose.heading.toDouble()));
            telemetry.addLine();

            displayCurrentPIDGains();

            telemetry.addLine();
            telemetry.addLine("Controls:");
            telemetry.addLine("A: Straight " + STRAIGHT_DISTANCE + "\" test");
            telemetry.addLine("B: Strafe " + STRAFE_DISTANCE + "\" test");
            telemetry.addLine("Y: Turn " + TURN_ANGLE + "° test");
            telemetry.addLine("X: Reset pose");
            telemetry.addLine("Sticks: Manual drive");

            telemetry.update();
        }
    }
}
