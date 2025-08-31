package org.firstinspires.ftc.teamcode.roadrunner;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.opMode;

import static java.lang.Math.PI;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.movement.Pose;

/**
 * OTOS Calibration and Test OpMode
 * Tests SparkFun OTOS sensor and displays real-time position tracking
 */
@TeleOp(name="OTOS Calibration Test", group="Tuning")
public class OTOSCalibrationTest extends LinearOpMode {

    private SparkFunOTOSLocalizer localizer;
    private ElapsedTime timer = new ElapsedTime();

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initializing OTOS...");
        telemetry.update();

        try {

            // Initialize OTOS localizer
            localizer = new SparkFunOTOSLocalizer(hardwareMap, new Pose2d(0, 0, 0));


            telemetry.addData("Status", "OTOS Initialized Successfully!");
            telemetry.addLine() ;
            telemetry.addLine("This OpMode will:");
            telemetry.addLine("1. Show real-time position tracking");
            telemetry.addLine("2. Test OTOS calibration");
            telemetry.addLine("3. Monitor OTOS sensor status");
            telemetry.addLine();
            telemetry.addLine("Manually push/rotate robot to test tracking");
            telemetry.addLine("Press PLAY to start");

        } catch (Exception e) {
            telemetry.addData("ERROR", "Failed to initialize OTOS");
            telemetry.addData("Error Message", e.getMessage());
            telemetry.addLine();
            telemetry.addLine("Check:");
            telemetry.addLine("1. OTOS is connected to I2C port");
            telemetry.addLine("2. Hardware config has 'otos' device");
            telemetry.addLine("3. OTOS has power (3.3V)");
        }

        telemetry.update();
        waitForStart();

        if (opModeIsActive() && localizer != null) {
            timer.reset();
            Pose2d startPose = new Pose2d(0, 0, 0);
            localizer.resetPose(startPose);

            while (opModeIsActive()) {
                // Update pose from OTOS
                Pose2d currentPose = localizer.update();

                // Display current position
                telemetry.addData("Runtime", "%.1f seconds", timer.seconds());
                telemetry.addLine();
                telemetry.addData("Current Position", "");
                telemetry.addData("  X (inches)", "%.2f", currentPose.position.x);
                telemetry.addData("  Y (inches)", "%.2f", currentPose.position.y);
                telemetry.addData("  Heading (degrees)", "%.1f", Math.toDegrees(currentPose.heading.toDouble()));
                telemetry.addLine();

                // Calculate distance from start
                double distanceFromStart = Math.sqrt(
                        currentPose.position.x * currentPose.position.x +
                                currentPose.position.y * currentPose.position.y
                );
                telemetry.addData("Distance from Start", "%.2f inches", distanceFromStart);

                // Get velocity info
                try {
                    PoseVelocity2d velocity = localizer.getPoseVelocity();
                    double speed = Math.sqrt(
                            velocity.linearVel.x * velocity.linearVel.x +
                                    velocity.linearVel.y * velocity.linearVel.y
                    );
                    telemetry.addData("Current Speed", "%.2f in/s", speed);
                    telemetry.addData("Angular Velocity", "%.1f deg/s", Math.toDegrees(velocity.angVel));
                } catch (Exception e) {
                    telemetry.addData("Velocity", "Unable to read");
                }

                telemetry.addLine();
                telemetry.addLine("Test Instructions:");
                telemetry.addLine("1. Push robot forward - X should increase");
                telemetry.addLine("2. Push robot right - Y should decrease");
                telemetry.addLine("3. Rotate robot left - Heading should increase");
                telemetry.addLine();
                telemetry.addLine("If directions are wrong, check OTOS mounting/offset");

                telemetry.update();
                sleep(50); // Update at ~20Hz
            }
        }
    }
}