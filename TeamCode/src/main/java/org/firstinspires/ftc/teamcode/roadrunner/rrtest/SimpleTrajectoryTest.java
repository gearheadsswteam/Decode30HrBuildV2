package org.firstinspires.ftc.teamcode.roadrunner.rrtest;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

@Autonomous(name = "Simple - Straight Line")
public class SimpleTrajectoryTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

        waitForStart();

        if (opModeIsActive()) {
            // Simple 24 inch forward trajectory
            Action trajectory = drive.actionBuilder(new Pose2d(0, 0, 0))
                    .lineToX(24)
                    .build();

            // Run it
            while (!isStopRequested() && trajectory.run(new TelemetryPacket())) {
                drive.updatePoseEstimate();
                Pose2d pose = drive.getPose();
                telemetry.addData("Position", "%.1f, %.1f", pose.position.x, pose.position.y);
                telemetry.update();
            }

            // Show result
            Pose2d finalPose = drive.getPose();
            telemetry.addData("Final X", "%.1f (target: 24.0)", finalPose.position.x);
            telemetry.update();
        }
    }
}
