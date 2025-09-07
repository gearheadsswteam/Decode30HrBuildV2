package org.firstinspires.ftc.teamcode.actionparts.testopmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.actionparts.FlywheelShooterWithRPMControl;
import org.firstinspires.ftc.teamcode.actionparts.ShooterTuning;

/**
 * Tuning Process
 *
 *
 * Here’s a **clear, repeatable, on-robot process** to tune your dual-motor flywheel (with `ShooterTuning` + the step-RPM TeleOp you have). Follow it in order—don’t skip ahead.
 *
 * ---
 *
 * # 0) Prep (5 min)
 *
 * * **Balls:** Use 6–10 balls in similar condition to competition ones. Retire torn/waterlogged balls.
 * * **Battery:** Freshly charged; warm up drivetrain and shooter for 60–90 sec so voltage and bearings stabilize.
 * * **Mechanics:** Verify:
 *
 *   * Wheels match (diameter, tread, wear), are balanced, and axles/bearings are rigid (no flex).
 *   * Hood path is rigid, smooth, and gives \~**5–10% compression** of ball diameter all along the contact arc.
 *   * Motors counter-rotate correctly (flip `MOTOR_*_REVERSED` if needed).
 *
 * ---
 *
 * # 1) Instrumentation (2 min)
 *
 * * Open **FTC Dashboard** → enable telemetry you already send: `targetRPM`, `currentRPM`, “ready”.
 * * Add these to your notes for each shot/burst:
 *
 *   * **ΔRPM dip** (target − minimum during shot)
 *   * **Recovery time** (to get back within `RECOVERY_BAND_RPM`)
 *   * **Hit/miss pattern** at your target distance
 *
 * ---
 *
 * # 2) Find a good **kF\_base** (feedforward) — P=I=D=0 first (5–10 min)
 *
 * 1. In code/Dashboard set `kP=0, kI=0, kD=0`. Keep voltage compensation ON (you already do `12/Voltage`).
 * 2. Using your D-pad (±50), test a few setpoints: e.g., **2200, 2700, 3200, 3700 RPM**.
 * 3. For each, when stable (no ball), compare `currentRPM` vs `targetRPM`:
 *
 *    * If **current < target** → bump **`kF_base` up** slightly.
 *    * If **current > target** → lower **`kF_base`** slightly.
 * 4. Goal: **steady-state error ≤ \~50–80 RPM** across those setpoints *without* P/I/D.
 *
 * > Tip: If `kF_base` looks right at 3200 but low at 2200 and high at 3700, your losses change with speed. Pick the middle as the priority (your most common shot) and let P handle the rest.
 *
 * ---
 *
 * # 3) Add **kP** for stiffness (3–6 min)
 *
 * 1. Raise **`kP`** in small steps.
 * 2. Watch the RPM trace when you *tap* the feeder (dry-fire with gate closed) or gently drag a cloth on a wheel:
 *
 *    * Increase `kP` until you see small oscillations/buzz around the setpoint, **then back off \~20%**.
 * 3. You should see a **smaller dip** and **faster return** when a ball is actually shot.
 *
 * ---
 *
 * # 4) Add a **little kI** only if needed (2–4 min)
 *
 * * If a constant **steady-state error** remains (not fixed by `kP`), add a *tiny* **`kI`** until it disappears.
 * * Too much **I** → sluggish response, overshoot after shots. Keep it small.
 *
 * > In FTC flywheels, **kD** is usually unnecessary; add only if you see overshoot you can’t tame with `kP`/`kI`.
 *
 * ---
 *
 * # 5) Calibrate **feeder timing** (2–4 min)
 *
 * * Set `FEED_DELAY_MS` just long enough to pass **one ball** consistently (start \~**120 ms**, adjust).
 * * If you see **big RPM dips** at the moment of feed, shorten open time, reduce friction at entry, or slightly reduce compression near the feeder hand-off.
 *
 * ---
 *
 * # 6) Set **bands & gating** (2–3 min)
 *
 * * Start with `READY_BAND_RPM = 50`, `RECOVERY_BAND_RPM = 75`.
 * * Verify your “ready” light only turns on when RPM is stable (no gradual climb).
 * * **Burst test (3 balls)**: Require `hasRecoveredSinceShot()` before the next feed.
 *
 *   * Good target: **dip ≤ 10–12%** of setpoint, **recovery ≤ 250–350 ms**, **shot-to-shot ΔRPM ≤ 60**.
 *
 * ---
 *
 * # 7) Build a **range→RPM map** (10–20 min)
 *
 * 1. On the field, mark typical distances you’ll shoot from (e.g., 24", 36", 48", 60").
 * 2. At each mark:
 *
 *    * Step RPM in **±50** increments until you **swish 8/10**.
 *    * Record the RPM and hood angle (if adjustable).
 * 3. Save these as **Dashboard presets** or just keep stepping with the D-pad on the day.
 *
 * > Quick physics sanity check (optional): wheel surface speed
 * > $v \approx \frac{\pi \cdot D \cdot \text{RPM}}{60}$
 * > Bigger D or higher RPM → longer range, but more sensitivity; tune for your sweet spot.
 *
 * ---
 *
 * # 8) Grouping & accuracy test (5–10 min)
 *
 * * From your **most common spot**, fire **10 shots** with your “ready” gate enforced:
 *
 *   * **Goal:** 9/10 in and a tight cluster. If not:
 *
 *     * **High/low misses:** tweak RPM ±50 or adjust hood angle 1–2°.
 *     * **Side hooks:** equalize compression left/right, check wheel directions and tread symmetry.
 *     * **First shot short, others fine:** increase inertia (small flywheel disk), add a bit more `kP`, or increase time to first feed after spin-up.
 *
 * ---
 *
 * # 9) Multi-ball stress test (5 min)
 *
 * * Rapid **5–7 ball** burst with recovery gating ON.
 * * Accept if: **last-ball arc ≈ first-ball arc**, RPM trace shows consistent dips and recoveries, no “creep” in average RPM.
 * * If later shots go short: increase inertia, reduce compression, raise `RECOVERY_BAND_RPM` strictness, or give 50–100 ms more time between feeds.
 *
 * ---
 *
 * # 10) Lock in **match presets** (2–3 min)
 *
 * * In `ShooterTuning`, store:
 *
 *   * **Close / Mid / Far** RPMs,
 *   * Feeder positions,
 *   * Motor reverse flags (as you ended up).
 * * Keep the D-pad stepping for fine trims on the field.
 *
 * ---
 *
 * # 11) Game-day maintenance
 *
 * * Warm up for 60–90 sec before auto.
 * * Swap wheels if chunks are missing; wipe the hood path.
 * * If switching to older balls: expect **\~50–150 RPM** higher for same arc; re-verify quickly with your D-pad.
 *
 * ---
 *
 * ## Quick “If X, then change Y” cheats
 *
 * * **All shots short:** +`kF_base` (or −compression/drag).
 * * **Oscillates at speed:** −`kP` (and/or −`kI`), check wheel balance.
 * * **First shot short only:** +inertia or +`kP`, stricter recovery gate, +warm-up time.
 * * **Bursts drift lower:** increase time between feeds a hair, add inertia, or reduce compression.
 * * **Sideways curve:** flip one motor direction, equalize compression/tread, check hood symmetry.
 *
 * ---
 *
 * If you tell me **wheel diameter/material**, **ball diameter**, **compression gap**, and **typical shot distance**, I’ll propose starting RPM presets and a first-pass PIDF so you’re 90% there on the first try.
 *
 */



@TeleOp(name="Flywheel PIDF Demo (Dashboard + Steps)", group="Shooter")
public class FlywheelTeleopDemo extends LinearOpMode {

    private FlywheelShooterWithRPMControl shooter;
    private Servo feeder;

    private double target = 0;
    private boolean dpadUpPrev = false, dpadDownPrev = false;

    @Override
    public void runOpMode() throws InterruptedException {
        shooter = new FlywheelShooterWithRPMControl(hardwareMap, "flyA", "flyB");
        feeder  = hardwareMap.get(Servo.class, "feeder");
        feeder.setPosition(ShooterTuning.FEED_CLOSED);

        FtcDashboard dash = FtcDashboard.getInstance();

        waitForStart();

        while (opModeIsActive()) {
            // --- Step RPM with D-pad ---
            if (gamepad1.dpad_up && !dpadUpPrev) {
                target = Math.min(target + ShooterTuning.STEP_SIZE_RPM, ShooterTuning.MAX_RPM);
            }
            if (gamepad1.dpad_down && !dpadDownPrev) {
                target = Math.max(target - ShooterTuning.STEP_SIZE_RPM, ShooterTuning.MIN_RPM);
            }
            dpadUpPrev = gamepad1.dpad_up;
            dpadDownPrev = gamepad1.dpad_down;

            // X to stop
            if (gamepad1.x) target = 0;

            shooter.applyVoltageCompensatedPIDF();
            shooter.setTargetRpm(target);

            boolean ready = shooter.isReadyToFire();

            // Right bumper to fire one ball when ready
            if (gamepad1.right_bumper && ready) {
                feeder.setPosition(ShooterTuning.FEED_OPEN);
                sleep((long) ShooterTuning.FEED_DELAY_MS);
                feeder.setPosition(ShooterTuning.FEED_CLOSED);
                shooter.markShot();

                // Wait for recovery before allowing another shot (bumper must be released)
                while (opModeIsActive() && !gamepad1.right_bumper && !shooter.hasRecoveredSinceShot()) {
                    shooter.applyVoltageCompensatedPIDF();
                    TelemetryPacket rpkt = new TelemetryPacket();
                    rpkt.put("targetRPM", target);
                    rpkt.put("currentRPM", shooter.getCurrentRpm());
                    rpkt.put("ready", false);
                    dash.sendTelemetryPacket(rpkt);

                    telemetry.addData("Recovering", "RPM=%.0f / %.0f", shooter.getCurrentRpm(), target);
                    telemetry.update();
                    idle();
                }
            }

            // Telemetry + dashboard
            telemetry.addData("Target RPM", "%.0f", target);
            telemetry.addData("Current RPM", "%.0f", shooter.getCurrentRpm());
            telemetry.addData("Ready", ready);
            telemetry.update();

            TelemetryPacket pkt = new TelemetryPacket();
            pkt.put("targetRPM", target);
            pkt.put("currentRPM", shooter.getCurrentRpm());
            pkt.put("ready", ready);
            dash.sendTelemetryPacket(pkt);

            idle();
        }

        shooter.stop();
    }
}
