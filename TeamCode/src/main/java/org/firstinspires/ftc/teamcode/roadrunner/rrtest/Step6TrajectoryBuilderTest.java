package org.firstinspires.ftc.teamcode.roadrunner.rrtest;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

@Autonomous(name = "Step 6: Trajectory Builder Test")
public class Step6TrajectoryBuilderTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

        telemetry.addLine("Ready to test TrajectoryActionBuilder");
        telemetry.update();
        waitForStart();

        if (opModeIsActive()) {
            // Build a simple trajectory
            Action trajectory = drive.actionBuilder(new Pose2d(0, 0, 0))
                    .lineToX(12)          // Drive forward 12 inches
                    .lineToY(12)          // Strafe left 12 inches
                    .turn(Math.toRadians(90))  // Turn 90 degrees
                    .lineToX(0)           // Drive back to X=0
                    .lineToY(0)           // Strafe back to Y=0
                    .turn(Math.toRadians(-90)) // Turn back to 0 degrees
                    .build();

            // Run the trajectory
            while (!isStopRequested() && trajectory.run(new TelemetryPacket())) {
                drive.updatePoseEstimate();
                Pose2d pose = drive.getPose();
                telemetry.addData("Current Position", "%.1f, %.1f, %.1f°",
                        pose.position.x, pose.position.y, Math.toDegrees(pose.heading.toDouble()));
                telemetry.update();
            }

            // Show final accuracy
            drive.updatePoseEstimate();
            Pose2d finalPose = drive.getPose();
            double error = Math.sqrt(
                    finalPose.position.x * finalPose.position.x +
                            finalPose.position.y * finalPose.position.y
            );

            telemetry.addLine("=== TRAJECTORY COMPLETE ===");
            telemetry.addData("Final Position", "%.2f, %.2f, %.1f°",
                    finalPose.position.x, finalPose.position.y, Math.toDegrees(finalPose.heading.toDouble()));
            telemetry.addData("Return Accuracy", "%.2f inches", error);

            if (error < 2.0) {
                telemetry.addLine("✓ EXCELLENT ACCURACY!");
            } else if (error < 4.0) {
                telemetry.addLine("✓ Good accuracy");
            } else {
                telemetry.addLine("⚠ Needs tuning");
            }

            telemetry.update();
        }
    }
}
