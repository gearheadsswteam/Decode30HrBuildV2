package org.firstinspires.ftc.teamcode.roadrunner.rrtest;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

@TeleOp(name = "Step 4: Simple Movements Test")
public class Step4SimpleMovementsTest extends LinearOpMode {
    private MecanumDrive drive;

    @Override
    public void runOpMode() {
        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.dpad_up) {
                // Move forward 1 inch per second
                drive.setDrivePowers(new PoseVelocity2d(new Vector2d(1.0, 0), 0));
                telemetry.addLine("Moving FORWARD");
            } else if (gamepad1.dpad_down) {
                // Move backward 1 inch per second
                drive.setDrivePowers(new PoseVelocity2d(new Vector2d(-1.0, 0), 0));
                telemetry.addLine("Moving BACKWARD");
            } else if (gamepad1.dpad_left) {
                // Strafe left 1 inch per second
                drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 1.0), 0));
                telemetry.addLine("Moving LEFT");
            } else if (gamepad1.dpad_right) {
                // Strafe right 1 inch per second
                drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, -1.0), 0));
                telemetry.addLine("Moving RIGHT");
            } else if (gamepad1.left_bumper) {
                // Rotate counterclockwise 30°/sec
                drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), Math.toRadians(30)));
                telemetry.addLine("Rotating LEFT");
            } else if (gamepad1.right_bumper) {
                // Rotate clockwise 30°/sec
                drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), Math.toRadians(-30)));
                telemetry.addLine("Rotating RIGHT");
            } else {
                // Stop
                drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));
                telemetry.addLine("STOPPED");
            }

            if (gamepad1.a) {
                drive.setPose(new Pose2d(0, 0, 0)); // Reset pose
            }

            drive.updatePoseEstimate();
            Pose2d pose = drive.getPose();
            telemetry.addData("X", "%.2f inches", pose.position.x);
            telemetry.addData("Y", "%.2f inches", pose.position.y);
            telemetry.addData("Heading", "%.1f degrees", Math.toDegrees(pose.heading.toDouble()));
            telemetry.addLine("A: Reset pose");
            telemetry.update();
        }
    }
}