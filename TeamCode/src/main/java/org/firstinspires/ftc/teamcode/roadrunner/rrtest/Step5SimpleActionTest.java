package org.firstinspires.ftc.teamcode.roadrunner.rrtest;


import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

@TeleOp(name = "Step 5: Simple Actions Test")

public class Step5SimpleActionTest extends LinearOpMode {
    private MecanumDrive drive;

    @Override
    public void runOpMode() {
        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

        telemetry.addLine("Ready to test simple actions");
        telemetry.update();
        waitForStart();

        if (opModeIsActive()) {
            // Test 1: Drive to position (12, 0, 0°)
            telemetry.addLine("Test 1: Drive forward 12 inches");
            telemetry.update();

            Action driveForward = drive.driveTo(new Pose2d(12, 0, 0));
            while (!isStopRequested() && driveForward.run(new TelemetryPacket())) {
                // Action is running
                drive.updatePoseEstimate();
                Pose2d pose = drive.getPose();
                telemetry.addData("Target", "12, 0, 0°");
                telemetry.addData("Current", "%.1f, %.1f, %.1f°",
                        pose.position.x, pose.position.y, Math.toDegrees(pose.heading.toDouble()));
                telemetry.update();
            }

            sleep(1000);

            // Test 2: Strafe to (12, 12, 0°)
            telemetry.addLine("Test 2: Strafe left 12 inches");
            telemetry.update();

            Action strafeLeft = drive.strafeTo(new Vector2d(12, 12));
            while (!isStopRequested() && strafeLeft.run(new TelemetryPacket())) {
                drive.updatePoseEstimate();
                Pose2d pose = drive.getPose();
                telemetry.addData("Target", "12, 12, 0°");
                telemetry.addData("Current", "%.1f, %.1f, %.1f°",
                        pose.position.x, pose.position.y, Math.toDegrees(pose.heading.toDouble()));
                telemetry.update();
            }

            sleep(1000);

            // Test 3: Turn to 90°
            telemetry.addLine("Test 3: Turn to 90 degrees");
            telemetry.update();

            Action turnLeft = drive.turnTo(Math.toRadians(90));
            while (!isStopRequested() && turnLeft.run(new TelemetryPacket())) {
                drive.updatePoseEstimate();
                Pose2d pose = drive.getPose();
                telemetry.addData("Target", "12, 12, 90°");
                telemetry.addData("Current", "%.1f, %.1f, %.1f°",
                        pose.position.x, pose.position.y, Math.toDegrees(pose.heading.toDouble()));
                telemetry.update();
            }

            // Final results
            drive.updatePoseEstimate();
            Pose2d finalPose = drive.getPose();
            telemetry.addLine("=== FINAL RESULTS ===");
            telemetry.addData("Final Position", "%.1f, %.1f, %.1f°",
                    finalPose.position.x, finalPose.position.y, Math.toDegrees(finalPose.heading.toDouble()));
            telemetry.update();
        }
    }
}
