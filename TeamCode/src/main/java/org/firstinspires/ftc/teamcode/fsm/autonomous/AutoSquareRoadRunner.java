package org.firstinspires.ftc.teamcode.fsm.opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Trajectory;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;

// Non-RR imports
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.MecanumDrive;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous(name = "Square Path Auto", group = "Autonomous")
public class SquarePathAuto extends LinearOpMode {

    @Override
    public void runOpMode() {
        // Initialize the drive
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        // Set the initial pose
        drive.setPoseEstimate(new Pose2d(0, 0, 0));

        // Build the 36-inch square trajectory sequence
        Trajectory traj1 = drive.trajectoryBuilder(new Pose2d(0, 0, 0))
                .forward(36)
                .build();

        Trajectory traj2 = drive.trajectoryBuilder(traj1.end())
                .strafeRight(36)
                .build();

        Trajectory traj3 = drive.trajectoryBuilder(traj2.end())
                .back(36)
                .build();

        Trajectory traj4 = drive.trajectoryBuilder(traj3.end())
                .strafeLeft(36)
                .build();

        // Wait for start
        waitForStart();

        if (isStopRequested()) return;

        // Follow each trajectory
        drive.followTrajectory(traj1);
        drive.followTrajectory(traj2);
        drive.followTrajectory(traj3);
        drive.followTrajectory(traj4);
    }
}
