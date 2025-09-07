package org.firstinspires.ftc.teamcode.actionparts;

import com.acmerobotics.dashboard.config.Config;

@Config
public class ShooterTuning {
    // --- PIDF base (will be voltage-compensated each loop) ---
    public static double kP = 0.0008;
    public static double kI = 0.0001;
    public static double kD = 0.0000;
    public static double kF_base = 0.11;   // tuned at ~12V

    // --- Bands & timings ---
    public static double READY_BAND_RPM    = 50;   // +/- to declare "ready"
    public static double RECOVERY_BAND_RPM = 75;   // +/- to declare "recovered" after shot
    public static double FEED_DELAY_MS     = 120;  // gate open time for one ball
    public static double MIN_SPINUP_MS     = 250;  // minimum time spinning before "ready"

    // --- Motor/encoder constants ---
    public static double TICKS_PER_REV     = 537.7; // set for your motor/gearbox

    // --- TeleOp controls (dashboard-tunable) ---
    public static int STEP_SIZE_RPM = 50;    // D-pad increment
    public static int MIN_RPM       = 0;
    public static int MAX_RPM       = 5000;  // clamp for safety

    // --- Feeder servo positions (tune to your hardware) ---
    public static double FEED_CLOSED = 0.05;
    public static double FEED_OPEN   = 0.35;

    // --- Motor direction (flip if your wheels fight each other) ---
    public static boolean MOTOR_A_REVERSED = false;
    public static boolean MOTOR_B_REVERSED = true;
}
