package org.firstinspires.ftc.teamcode.vision;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


/**
 * Simple test OpMode to verify Decode AprilTag detection & slot mapping.
 * X = toggle sticky hold (0ms vs 300ms)
 */
@TeleOp(name = "Decode Tag Test (2025–26)", group = "Vision")
public class DecodeTagTestOpMode extends LinearOpMode {
    @Override
    public void runOpMode() {
        AprilTagDecodeService tags = new AprilTagDecodeService(hardwareMap, "Webcam 1")
                .setExpectedIds(1, 2, 3) // TODO: set your real LEFT/CENTER/RIGHT IDs
                .setTagSizeMeters(0.165) // TODO: update to official size if different
                .setHoldMillis(300) // start with 300ms sticky memory
                .useSeasonLibraryOrFallback();
        tags.init();


        telemetry.addLine("Decode AprilTag ready. Press START.");
        telemetry.update();
        waitForStart();


        boolean sticky = true;
        while (opModeIsActive()) {
            if (gamepad1.x) { // toggle sticky hold for quick experimentation
                sticky = !sticky;
                tags.setHoldMillis(sticky ? 300 : 0);
            }


            tags.addTelemetry(telemetry);
            AprilTagDecodeService.Slot slot = tags.getSeenSlot();
            switch (slot) {
                case LEFT:
                    telemetry.addLine("Pick: LEFT");
                    break;
                case CENTER:
                    telemetry.addLine("Pick: CENTER");
                    break;
                case RIGHT:
                    telemetry.addLine("Pick: RIGHT");
                    break;
                default:
                    telemetry.addLine("Pick: NONE");
            }
            telemetry.addData("Sticky Hold", sticky ? "ON (300ms)" : "OFF");
            telemetry.update();
        }


        tags.close();
    }

}