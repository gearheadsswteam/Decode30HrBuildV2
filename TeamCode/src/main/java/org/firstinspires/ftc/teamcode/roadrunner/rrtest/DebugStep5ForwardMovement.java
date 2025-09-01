package org.firstinspires.ftc.teamcode.roadrunner.rrtest;

import static org.firstinspires.ftc.teamcode.fsm.hardware.ValueStorage.telemetry;
import static java.lang.Thread.sleep;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

@TeleOp(name = "Debug Step 5 - Forward Movement")
public class DebugStep5ForwardMovement extends LinearOpMode {
    private MecanumDrive drive;

    @Override
    public void runOpMode() {
        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

        telemetry.addLine("Step 5 Debug - Testing Forward Movement");
        telemetry.addLine("Press A to test driveTo() action");
        telemetry.addLine("Press B to test manual movement");
        telemetry.addLine("Press X to reset pose");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            drive.updatePoseEstimate();
            Pose2d currentPose = drive.getPose();

            if (gamepad1.a) {
                // Test the Action system (like Step 5)
                telemetry.addLine("Testing driveTo() Action...");
                telemetry.update();

                Action driveForward = drive.driveTo(new Pose2d(12, 0, 0));
                long startTime = System.currentTimeMillis();
                int iterations = 0;

                while (opModeIsActive() && driveForward.run(new TelemetryPacket())) {
                    iterations++;
                    drive.updatePoseEstimate();
                    Pose2d pose = drive.getPose();

                    long elapsed = System.currentTimeMillis() - startTime;
                    double error = Math.abs(12.0 - pose.position.x);

                    telemetry.addData("Target X", "12.0");
                    telemetry.addData("Current X", "%.2f", pose.position.x);
                    telemetry.addData("Error", "%.2f inches", error);
                    telemetry.addData("Time", "%.1f sec", elapsed / 1000.0);
                    telemetry.addData("Iterations", iterations);
                    telemetry.addData("Action Running", "YES");
                    telemetry.update();

                    // Safety timeout
                    if (elapsed > 10000) {
                        telemetry.addLine("TIMEOUT - Action took too long");
                        telemetry.update();
                        break;
                    }
                }

                telemetry.addLine("Action completed or stopped");
                telemetry.update();
                sleep(2000);
            }

            if (gamepad1.b) {
                // Test manual movement for comparison
                telemetry.addLine("Manual forward movement...");
                telemetry.update();

                Pose2d startPose = currentPose;
                long startTime = System.currentTimeMillis();

                while (gamepad1.b && opModeIsActive()) {
                    // Move forward at 6 inches/second
                    drive.setDrivePowers(new PoseVelocity2d(new Vector2d(6.0, 0), 0));
                    drive.updatePoseEstimate();

                    Pose2d pose = drive.getPose();
                    double distanceMoved = pose.position.x - startPose.position.x;
                    long elapsed = System.currentTimeMillis() - startTime;

                    telemetry.addData("Distance Moved", "%.2f inches", distanceMoved);
                    telemetry.addData("Time", "%.1f sec", elapsed / 1000.0);
                    telemetry.addData("Speed", "%.1f in/sec", distanceMoved / (elapsed / 1000.0));
                    telemetry.addLine("Hold B to continue moving");
                    telemetry.update();
                }

                // Stop
                drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));
            }

            if (gamepad1.x) {
                drive.setPose(new Pose2d(0, 0, 0));
                telemetry.addLine("Pose reset to (0, 0, 0)");
                telemetry.update();
                sleep(500);
            }

            // Show current status
            telemetry.addData("Current X", "%.2f", currentPose.position.x);
            telemetry.addData("Current Y", "%.2f", currentPose.position.y);
            telemetry.addData("Current Heading", "%.1f°", Math.toDegrees(currentPose.heading.toDouble()));
            telemetry.addLine();
            telemetry.addLine("A: Test driveTo() action");
            telemetry.addLine("B: Manual forward (hold)");
            telemetry.addLine("X: Reset pose");
            telemetry.update();
        }
    }
}
