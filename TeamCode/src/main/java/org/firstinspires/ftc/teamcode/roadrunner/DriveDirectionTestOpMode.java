package org.firstinspires.ftc.teamcode.roadrunner;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Disabled
@TeleOp(name = "Drive Direction Test", group = "Test")
public class DriveDirectionTestOpMode extends OpMode {

    private MecanumDrive drive;

    @Override
    public void init() {
        Pose2d startPose = new Pose2d(0, 0, 0);
        drive = new MecanumDrive(hardwareMap, startPose);
    }

    @Override
    public void loop() {
        // Default stop
        PoseVelocity2d cmd = new PoseVelocity2d(new Vector2d(0, 0), 0);

        if (gamepad1.a) {
            // Forward
            cmd = new PoseVelocity2d(new Vector2d(0.5, 0), 0);
        } else if (gamepad1.b) {
            // Backward
            cmd = new PoseVelocity2d(new Vector2d(-0.5, 0), 0);
        } else if (gamepad1.x) {
            // Strafe Left
            cmd = new PoseVelocity2d(new Vector2d(0, 0.5), 0);
        } else if (gamepad1.y) {
            // Strafe Right
            cmd = new PoseVelocity2d(new Vector2d(0, -0.5), 0);
        } else if (gamepad1.left_bumper) {
            // Turn CCW
            cmd = new PoseVelocity2d(new Vector2d(0, 0), 0.5);
        } else if (gamepad1.right_bumper) {
            // Turn CW
            cmd = new PoseVelocity2d(new Vector2d(0, 0), -0.5);
        }

        drive.setDrivePowers(cmd);

        // Telemetry to confirm localizer + heading
        telemetry.addData("X", drive.getPose().position.x);
        telemetry.addData("Y", drive.getPose().position.y);
        telemetry.addData("Heading (deg)", Math.toDegrees(drive.getPose().heading.toDouble()));
        telemetry.update();
    }
}
