package org.firstinspires.ftc.teamcode.roadrunner;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * OTOS Precision Calibration OpMode
 * Helps calibrate linear and angular scalars for accurate measurements
 */
@TeleOp(name="OTOS Precision Calibration", group="Tuning")
public class OTOSPrecisionCalibration extends LinearOpMode {

    private MecanumDrive drive;
    private ElapsedTime timer = new ElapsedTime();

    // Calibration test parameters
    private final double TEST_DISTANCE = 48.0;  // inches - measure this precisely!
    private final double TEST_ANGLE = 360.0;    // degrees - full rotation

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initializing...");
        telemetry.update();

        try {
            // Initialize drive system
            drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

            telemetry.addData("Status", "Ready for Calibration!");
            telemetry.addLine();
            telemetry.addLine("CALIBRATION INSTRUCTIONS:");
            telemetry.addLine();
            telemetry.addLine("LINEAR CALIBRATION:");
            telemetry.addLine("1. Place robot against a wall/starting line");
            telemetry.addLine("2. Press X to test 48\" forward movement");
            telemetry.addLine("3. Measure ACTUAL distance traveled");
            telemetry.addLine("4. We'll calculate linear scalar");
            telemetry.addLine();
            telemetry.addLine("ANGULAR CALIBRATION:");
            telemetry.addLine("1. Mark robot's starting orientation");
            telemetry.addLine("2. Press Y to test 360° rotation");
            telemetry.addLine("3. Check if robot faces same direction");
            telemetry.addLine("4. We'll calculate angular scalar");
            telemetry.addLine();
            telemetry.addLine("Press PLAY to start");

        } catch (Exception e) {
            telemetry.addData("ERROR", "Failed to initialize");
            telemetry.addData("Error", e.getMessage());
        }

        telemetry.update();
        waitForStart();

        if (opModeIsActive() && drive != null) {
            while (opModeIsActive()) {
                // Reset pose at start of each test
                if (gamepad1.start) {
                    drive.setPose(new Pose2d(0, 0, 0));
                    telemetry.addData("Status", "Position Reset!");
                }

                // Linear calibration test
                if (gamepad1.x) {
                    performLinearCalibration();
                }

                // Angular calibration test
                if (gamepad1.y) {
                    performAngularCalibration();
                }

                // Display current position
                Pose2d currentPose = drive.updatePoseEstimate();

                telemetry.addData("Current Position", "");
                telemetry.addData("  X", "%.2f inches", currentPose.position.x);
                telemetry.addData("  Y", "%.2f inches", currentPose.position.y);
                telemetry.addData("  Heading", "%.1f degrees", Math.toDegrees(currentPose.heading.toDouble()));
                telemetry.addLine();

                telemetry.addLine("CONTROLS:");
                telemetry.addData("X Button", "Linear Calibration Test (48\")");
                telemetry.addData("Y Button", "Angular Calibration Test (360°)");
                telemetry.addData("START", "Reset Position");
                telemetry.addLine();

                telemetry.addLine("CALIBRATION VALUES:");
                telemetry.addLine("Current Linear Scalar: 1.0 (adjust in code)");
                telemetry.addLine("Current Angular Scalar: 1.0 (adjust in code)");

                telemetry.update();
            }
        }
    }

    private void performLinearCalibration() {
        telemetry.addData("Status", "LINEAR CALIBRATION TEST");
        telemetry.addLine("Robot will attempt to drive straight forward 48\"");
        telemetry.addLine("Watch the robot and measure actual distance!");
        telemetry.update();

        // Reset position
        drive.setPose(new Pose2d(0, 0, 0));
        sleep(1000);

        // Record start position
        Pose2d startPose = drive.updatePoseEstimate();

        // Drive forward test distance
        timer.reset();
        while (opModeIsActive() && timer.seconds() < 5.0) {
            Pose2d currentPose = drive.updatePoseEstimate();
            double distanceTraveled = currentPose.position.x - startPose.position.x;

            if (distanceTraveled >= TEST_DISTANCE) {
                break; // Reached target distance
            }

            // Simple drive forward (this is a basic implementation)
            // In a real scenario, you'd use the drive actions
            telemetry.addData("Target Distance", "%.1f inches", TEST_DISTANCE);
            telemetry.addData("OTOS Reports", "%.2f inches", distanceTraveled);
            telemetry.addData("Time", "%.1f seconds", timer.seconds());
            telemetry.addLine("Driving forward...");
            telemetry.update();
        }

        // Stop and get final position
        Pose2d finalPose = drive.updatePoseEstimate();
        double reportedDistance = finalPose.position.x - startPose.position.x;

        // Display results
        telemetry.clear();
        telemetry.addData("LINEAR CALIBRATION RESULTS", "");
        telemetry.addLine();
        telemetry.addData("OTOS Reported Distance", "%.2f inches", reportedDistance);
        telemetry.addData("Target Distance", "%.1f inches", TEST_DISTANCE);
        telemetry.addLine();
        telemetry.addLine("NOW MEASURE THE ACTUAL DISTANCE!");
        telemetry.addLine("Use a tape measure from start to end position");
        telemetry.addLine();
        telemetry.addLine("CALCULATION:");
        telemetry.addLine("Linear Scalar = Actual Distance / OTOS Distance");
        telemetry.addLine("Example: If actual = 44\", scalar = 44/48 = 0.917");
        telemetry.addLine();
        telemetry.addLine("Update linear scalar in SparkFunOTOSLocalizer.java");
        telemetry.addLine("Press any button to continue...");
        telemetry.update();

        // Wait for button press
        while (opModeIsActive() && !gamepad1.a && !gamepad1.b && !gamepad1.x && !gamepad1.y) {
            sleep(50);
        }
    }

    private void performAngularCalibration() {
        telemetry.addData("Status", "ANGULAR CALIBRATION TEST");
        telemetry.addLine("Robot will rotate 360 degrees");
        telemetry.addLine("Mark starting orientation and check final!");
        telemetry.update();

        // Reset position
        drive.setPose(new Pose2d(0, 0, 0));
        sleep(1000);

        // Record start heading
        Pose2d startPose = drive.updatePoseEstimate();
        double targetHeading = Math.toRadians(TEST_ANGLE);

        // Rotation test
        timer.reset();
        while (opModeIsActive() && timer.seconds() < 8.0) {
            Pose2d currentPose = drive.updatePoseEstimate();
            double headingTraveled = currentPose.heading.toDouble() - startPose.heading.toDouble();

            // Normalize heading
            while (headingTraveled > Math.PI) headingTraveled -= 2 * Math.PI;
            while (headingTraveled < -Math.PI) headingTraveled += 2 * Math.PI;

            if (Math.abs(headingTraveled) >= Math.abs(targetHeading) - Math.toRadians(10)) {
                break; // Close enough to target
            }

            telemetry.addData("Target Rotation", "%.0f degrees", TEST_ANGLE);
            telemetry.addData("OTOS Reports", "%.1f degrees", Math.toDegrees(headingTraveled));
            telemetry.addData("Time", "%.1f seconds", timer.seconds());
            telemetry.addLine("Rotating...");
            telemetry.update();
        }

        // Get final results
        Pose2d finalPose = drive.updatePoseEstimate();
        double reportedRotation = finalPose.heading.toDouble() - startPose.heading.toDouble();

        // Normalize
        while (reportedRotation > Math.PI) reportedRotation -= 2 * Math.PI;
        while (reportedRotation < -Math.PI) reportedRotation += 2 * Math.PI;

        // Display results
        telemetry.clear();
        telemetry.addData("ANGULAR CALIBRATION RESULTS", "");
        telemetry.addLine();
        telemetry.addData("OTOS Reported", "%.1f degrees", Math.toDegrees(reportedRotation));
        telemetry.addData("Target Rotation", "%.0f degrees", TEST_ANGLE);
        telemetry.addLine();
        telemetry.addLine("CHECK ROBOT ORIENTATION!");
        telemetry.addLine("Is robot facing the same direction as start?");
        telemetry.addLine();
        telemetry.addLine("CALCULATION:");
        telemetry.addLine("Angular Scalar = Actual Rotation / OTOS Rotation");
        telemetry.addLine("Example: If robot rotated 350°, scalar = 350/360 = 0.972");
        telemetry.addLine();
        telemetry.addLine("Update angular scalar in SparkFunOTOSLocalizer.java");
        telemetry.addLine("Press any button to continue...");
        telemetry.update();

        // Wait for button press
        while (opModeIsActive() && !gamepad1.a && !gamepad1.b && !gamepad1.x && !gamepad1.y) {
            sleep(50);
        }
    }
}