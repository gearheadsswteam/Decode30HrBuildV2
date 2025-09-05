package org.firstinspires.ftc.teamcode.otos;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.drive.MecanumDrive;

@Autonomous(name = "OTOS Test", group = "Test")
public class OTOSTestOpMode extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize drive
        MecanumDrive drive = new MecanumDrive(hardwareMap);

        // Set starting pose
        drive.setPose(new Pose2d(0, 0, 0));

        telemetry.addData("Status", "Initialized");
        telemetry.addLine("Press start to begin OTOS test");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Update localizer
            drive.updatePoseEstimate();

            // Get current pose
            Pose2d pose = drive.getPose();

            // Display telemetry
            telemetry.addData("X Position", "%.2f inches", pose.position.x);
            telemetry.addData("Y Position", "%.2f inches", pose.position.y);
            telemetry.addData("Heading", "%.2f degrees", Math.toDegrees(pose.heading.toDouble()));

            // Get velocity if available
            try {
                PoseVelocity2d velocity = drive.getVelocity();
                telemetry.addData("X Velocity", "%.2f in/s", velocity.linearVel.x);
                telemetry.addData("Y Velocity", "%.2f in/s", velocity.linearVel.y);
                telemetry.addData("Angular Velocity", "%.2f deg/s", Math.toDegrees(velocity.angVel));
            } catch (Exception e) {
                telemetry.addData("Velocity", "Not available");
            }

            telemetry.addLine();
            telemetry.addLine("Move robot around to test tracking");
            telemetry.addLine("Press X to reset position");

            // Reset position if X is pressed
            if (gamepad1.x) {
                drive.setPose(new Pose2d(0, 0, 0));
                telemetry.addLine("Position reset!");
            }

            telemetry.update();

            sleep(50); // Update at 20Hz
        }
    }
}