package org.firstinspires.ftc.teamcode.drive;

public class DriveConstants {
    // Feedforward (start rough; tune using RR tuners)
    public static double kV = 1.0 / 40.0; // 1 / max vel (in/s)
    public static double kA = 0.0;
    public static double kStatic = 0.0;

    // Robot geometry (inches)
    public static double TRACK_WIDTH = 14.0;
    public static double WHEEL_BASE = 14.0;
    public static double LATERAL_MULTIPLIER = 1.0;

    // Constraints
    public static double MAX_VEL = 40.0;        // in/s
    public static double MAX_ACCEL = 40.0;      // in/s²
    public static double MAX_ANG_VEL = Math.toRadians(180);
    public static double MAX_ANG_ACCEL = Math.toRadians(180);
}
