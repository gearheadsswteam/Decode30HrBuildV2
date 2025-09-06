package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.autonomous.MecanumAutoAPI;

@Autonomous(name = "AutoAPI Demo: Mecanum Basic Moves", group = "Auto")
public class AutoAPIDemoOpMode extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        // Set hub orientation (edit to match your Control Hub mounting!)
        RevHubOrientationOnRobot hubOrientation = MecanumAutoAPI.defaultREVHubOrientation();

        // Create the API. Replace motor names with your config names.
        MecanumAutoAPI.Params p = new MecanumAutoAPI.Params();
        // Example: if you have 96mm wheels -> 3.78in
        // p.wheelDiameterIn = 3.78;
        // p.ticksPerMotorRev = 383.6; // if using 435RPM Yellow Jacket
        // p.lateralMultiplier = 1.16; // tune this on your field

        MecanumAutoAPI api = new MecanumAutoAPI(
                this, hardwareMap,
                "front_left", "front_right", "back_left", "back_right",
                hubOrientation,
                p
        );

        // Show calibration numbers (CPI, geometry, etc.) on DS before start
        api.logCalibrationTelemetry();

        telemetry.addLine("Initialized. Waiting for start...");
        telemetry.update();
        waitForStart();
        if (isStopRequested()) return;

        // === Example path ===
        api.forward(24, 0.6, 5.0);       // forward 24"
        api.strafeRight(12, 0.6, 5.0);   // strafe right 12"
        api.turnLeft(90, 4.0);           // left turn 90 deg
        api.backward(12, 0.6, 4.0);      // back 12"

        telemetry.addLine("Path complete");
        telemetry.update();
        sleep(1000);
    }
}

// ================= QUICK TUNING CHECKLIST =================
// 1) Set ticksPerMotorRev to match your motors (e.g., 537.6 for goBILDA 312RPM; 383.6 for 435RPM; 28 for NeveRest 40)
// 2) Measure wheelDiameterIn in inches (tread to tread). Use calipers if possible.
// 3) Confirm motor directions make +Y forward. If forward() goes backward, flip all directions or swap signs.
// 4) Tune lateralMultiplier: command 24" strafe; adjust until measured distance matches.
// 5) IMU orientation MUST match your hub mounting. Update defaultREVHubOrientation() accordingly.
// 6) Turn PID: if overshooting, raise kD a bit or reduce kP; if sluggish, raise kP or minTurnPower slightly.
// 7) Use fresh batteries; low voltage skews behavior.
