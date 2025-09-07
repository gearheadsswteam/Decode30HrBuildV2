package org.firstinspires.ftc.teamcode.actionparts.testopmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.actionparts.FlyWheelController;

/**
 * Test flywheel with RPM controller, tuned live via FTCDashboard
 *
 * 🛠 Step 1 – Safety & Setup
 *
 * Make sure your robot is stable on the field tile (wheels won’t slip).
 *
 * Place a target goal or taped mark at the distance you want to shoot.
 *
 * Fully charge your battery — low voltage will skew results.
 *
 * Use FTCDashboard telemetry to monitor target RPM and actual RPM.
 *
 * 🎯 Step 2 – RPM Validation (No Balls Yet)
 *
 * Run your flywheel at a chosen targetRPM (say 3000 RPM).
 *
 * Watch Dashboard telemetry for actual RPM.
 *
 * It should settle close to target (±100 RPM is usually OK).
 *
 * If it oscillates: lower kP.
 *
 * If it lags or never reaches target: raise kP.
 *
 * Test multiple setpoints (2000, 2500, 3000, 3500 RPM) to make sure control loop behaves smoothly.
 *
 * 🏐 Step 3 – Single Ball Testing
 *
 * Load one ball at a time into the flywheel.
 *
 * Fire at your target while logging actual RPM.
 *
 * Watch for:
 *
 * Distance: does it land short/long?
 *
 * Consistency: do multiple shots land in the same area?
 *
 * Recovery: after a shot, does the RPM dip and then recover fast (<0.5s)?
 *
 * 👉 If shots are short → increase targetRPM.
 * 👉 If shots are overshooting → decrease targetRPM.
 * 👉 If recovery is slow → increase kP slightly, or check if your flywheel has enough inertia.
 *
 * 📊 Step 4 – Systematic Distance Calibration
 *
 * Place markers at 1 ft intervals on the field (e.g., 6 ft, 8 ft, 10 ft).
 *
 * Shoot 3–5 balls at each distance with a fixed RPM.
 *
 * Record the average landing point.
 *
 * Build a table like:
 *
 * Target Distance (ft)	Best RPM
 * 6 ft	2400 RPM
 * 8 ft	2800 RPM
 * 10 ft	3200 RPM
 * 🔧 Step 5 – Accuracy Tuning
 *
 * Adjust RPM in small steps (±50–100 RPM) and retest.
 *
 * Try different ball types/conditions (new, worn, squishy) to see variation.
 *
 * If variation is large, you may need:
 *
 * More flywheel inertia (heavier wheel).
 *
 * Double flywheel design (top + bottom).
 *
 * Better feeder consistency.
 *
 * 🧪 Step 6 – Stress Test
 *
 * Fire a full magazine (5–10 balls) as fast as your intake feeds.
 *
 * Monitor if RPM holds close to target between shots.
 *
 * If it dips too much, you may need higher kP or even a feed delay so flywheel recovers before next shot.
 *
 * 🏆 Step 7 – Finalize
 *
 * Pick 1–2 standard RPMs (e.g., “short shot” and “long shot”).
 *
 * Hard-code those in your TeleOp with gamepad buttons.
 *
 * Keep tuning table in your engineering notebook for Worlds-style scouting.
 */
@Config
@TeleOp(name = "Flywheel Dashboard Test", group = "Test")
public class FlywheelDashboardTestOpMode extends OpMode {

    private FlyWheelController flywheel;

    // These fields will show up in Dashboard "Config" tab
    public static double targetRPM = 3000;   // change live in Dashboard
    public static double kP = 0.0005;        // tune live in Dashboard

    private FtcDashboard dashboard;

    @Override
    public void init() {
        DcMotorEx left = hardwareMap.get(DcMotorEx.class, "leftFlywheel");
        DcMotorEx right = hardwareMap.get(DcMotorEx.class, "rightFlywheel");

        flywheel = new FlyWheelController(left, right, hardwareMap);

        // link dashboard instance
        dashboard = FtcDashboard.getInstance();

        telemetry.addLine("Flywheel Dashboard Test Initialized");
    }

    @Override
    public void loop() {
        // Set kP dynamically
        flywheel.setkP(kP);

        // Always apply target from dashboard
        if (targetRPM > 0) {
            flywheel.spinnersOn(targetRPM);
        } else {
            flywheel.spinnersOff();
        }

        // Update control loop
        flywheel.update(); //This should go in TeleOp while loop

        // Report telemetry (goes both to Driver Station and Dashboard)
        telemetry.addData("TargetRPM", targetRPM);
        telemetry.update();
    }
}
