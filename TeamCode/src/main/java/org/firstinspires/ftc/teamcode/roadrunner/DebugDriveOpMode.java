package org.firstinspires.ftc.teamcode.roadrunner;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Disabled
@TeleOp(name = "Debug Drive OpMode", group = "Test")
public class DebugDriveOpMode extends OpMode {
    private MecanumDrive drive;
    private Action currentAction;

    @Override
    public void init() {
        Pose2d startPose = new Pose2d(0, 0, 0);
        drive = new MecanumDrive(hardwareMap, startPose);

        telemetry.addLine("Debug Menu:");
        telemetry.addLine("A = Test forward motor powers");
        telemetry.addLine("B = Test strafe motor powers");
        telemetry.addLine("X = Show localizer pose");
        telemetry.addLine("Y = Run 24\" forward trajectory");
    }

    boolean poseDebugMode = false;

    @Override
    public void loop() {
        // === Step 1: Direct motor forward test ===
        if (gamepad1.a) {
            drive.setDrivePowers(new PoseVelocity2d(new Vector2d(1, 0), 0));
            telemetry.addLine("Driving forward with raw motor powers");
        }

        // === Step 2: Direct motor strafe test ===
        else if (gamepad1.b) {
            drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 1), 0));
            telemetry.addLine("Strafing right with raw motor powers");
        }


        // === Step 3: Localizer debug ===
        // Start pose debug when X is pressed
        if (gamepad1.x) {
            poseDebugMode = true;
        }

// Stop pose debug when B is pressed
        if (gamepad1.b) {
            poseDebugMode = false;
        }

        if (poseDebugMode) {
            Pose2d pose = drive.updatePoseEstimate();

            telemetry.addData("X", pose.position.x);
            telemetry.addData("Y", pose.position.y);
            telemetry.addData("Heading (deg)", Math.toDegrees(pose.heading.toDouble()));

            // keep motors stopped during pose debug
            drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));
        }

        telemetry.update();


        // === Step 4: Trajectory test ===
        if (gamepad1.y && currentAction == null) {
            Pose2d currentPose = drive.getPose();
            currentAction = drive.actionBuilder(currentPose)
                    .lineToX(currentPose.position.x + 24)
                    .build();
            telemetry.addLine("Starting trajectory: Forward 24\"");
        }

        // === Run trajectory if active ===
        if (currentAction != null) {
            TelemetryPacket packet = new TelemetryPacket();
            boolean stillRunning = currentAction.run(packet);

            telemetry.addData("Trajectory packet", packet.toString());

            if (!stillRunning) {
                currentAction = null;
                telemetry.addLine("Trajectory finished");
            }
        }

        telemetry.update();
    }
}
