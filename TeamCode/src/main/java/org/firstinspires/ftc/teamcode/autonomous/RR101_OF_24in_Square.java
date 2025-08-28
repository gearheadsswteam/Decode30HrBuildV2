package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.localization.OpticalFlowLocalizer;
import org.firstinspires.ftc.teamcode.sensors.SparkFunOpticalFlow;

import com.qualcomm.hardware.rev.RevIMU;

/**
 * RR 1.0.1 autonomous: drive a 24" square using SparkFun optical odometry (XY) + IMU (heading).
 * Path: forward 24 -> right 24 -> back 24 -> left 24.
 *
 * Requires:
 *  - Road Runner 1.0.1 (Quickstart)
 *  - Your OpticalFlowLocalizer + SparkFunOpticalFlow classes added
 *  - Two I2C sensors configured as "of_x" (0x2A) and "of_y" (0x2B), and an IMU named "imu"
 */
@Autonomous(name = "RR 1.0.1 + SparkFun OF: 24in Square", group = "RR")
public class RR101_OF_24in_Square extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        // Build RR drive
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        // Build optical odometry + IMU, install as the RR localizer
        SparkFunOpticalFlow ofX = new SparkFunOpticalFlow(hardwareMap, "of_x", 0x2A);
        SparkFunOpticalFlow ofY = new SparkFunOpticalFlow(hardwareMap, "of_y", 0x2B);
        RevIMU imu = hardwareMap.get(RevIMU.class, "imu");

        drive.setLocalizer(new OpticalFlowLocalizer(hardwareMap, ofX, ofY, imu));

        // Start pose (facing +X, heading 0 rad)
        Pose2d startPose = new Pose2d(0, 0, 0);
        drive.setPoseEstimate(startPose);

        final double SIDE_IN = 24.0;

        // RR 1.0.1 supports TrajectorySequence in the Quickstart
        var seq = drive.trajectorySequenceBuilder(startPose)
                .forward(SIDE_IN)
                .strafeRight(SIDE_IN)
                .back(SIDE_IN)
                .strafeLeft(SIDE_IN)
                .build();

        telemetry.addLine("Loaded: 24\" square (RR 1.0.1).");
        telemetry.update();
        waitForStart();
        if (isStopRequested()) return;

        drive.followTrajectorySequence(seq);

        // Report final pose estimated by optical odom + IMU
        Pose2d p = drive.getPoseEstimate();
        telemetry.addData("Final X (in)", "%.2f", p.getX());
        telemetry.addData("Final Y (in)", "%.2f", p.getY());
        telemetry.addData("Final Heading (deg)", "%.1f", Math.toDegrees(p.getHeading()));
        telemetry.update();
        sleep(1000);
    }
}
