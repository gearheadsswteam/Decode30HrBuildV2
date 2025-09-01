package org.firstinspires.ftc.teamcode.roadrunner;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "RR 1.0.1 Forward Test", group = "Test")
public class RR101ForwardTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        // Start pose at (0,0,0)
        Pose2d startPose = new Pose2d(0, 0, 0);
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);

        waitForStart();

        if (isStopRequested()) return;

        // Build trajectory: (0,0,0) → (24,0,0)
        Action action = drive.actionBuilder(startPose)
                .lineToX(24)
                .build();

        // Run trajectory in loop
        TelemetryPacket packet = new TelemetryPacket();
        while (opModeIsActive() && action.run(packet)) {
            telemetry.addLine("Running trajectory...");
            telemetry.update();
        }

        // Report final pose
        Pose2d finalPose = drive.getPose();
        telemetry.addData("Final X", finalPose.position.x);
        telemetry.addData("Final Y", finalPose.position.y);
        telemetry.addData("Final Heading", Math.toDegrees(finalPose.heading.toDouble()));
        telemetry.update();

        sleep(2000);
    }
}

