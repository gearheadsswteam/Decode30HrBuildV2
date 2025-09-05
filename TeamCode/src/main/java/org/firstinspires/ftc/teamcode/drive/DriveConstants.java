// 1. UPDATE DriveConstants.java - Add missing OTOS constants
package org.firstinspires.ftc.teamcode.drive;

public class DriveConstants {

    /*
     * Motor constants for GoBILDA 312 RPM Yellow Jacket motors
     */
    public static final double TICKS_PER_REV = 537.7;
    public static final double MAX_RPM = 312;

    /*
     * IMPORTANT: Set to false since you're using OTOS optical odometry
     */
    public static final boolean RUN_USING_ENCODER = false; // Using OTOS instead

    // Motor PID values (not used when RUN_USING_ENCODER = false)
    public static final double MOTOR_VELO_PID_P = 0.0;
    public static final double MOTOR_VELO_PID_I = 0.0;
    public static final double MOTOR_VELO_PID_D = 0.0;
    public static final double MOTOR_VELO_PID_F = 0.0;

    /*
     * Physical robot constants - UPDATE THESE BASED ON YOUR ROBOT
     */
    public static double WHEEL_RADIUS = 1.88976; // in (for drive wheels)
    public static double GEAR_RATIO = 1;
    public static double TRACK_WIDTH = 15.0; // in - NEEDS CALIBRATION

    // ADD THESE MISSING OTOS CONSTANTS:
    /*
     * OTOS (Optical Tracking Odometry Sensor) calibration constants
     * These need to be determined through testing
     */
    public static final double OTOS_X_TICKS_PER_INCH = 2000.0; // PLACEHOLDER - NEEDS CALIBRATION
    public static final double OTOS_Y_TICKS_PER_INCH = 2000.0; // PLACEHOLDER - NEEDS CALIBRATION

    // Legacy constants for compatibility (referenced in OpticalOdometryLocalizer)
    public static final double ODOM_X_TICKS_PER_INCH = OTOS_X_TICKS_PER_INCH;
    public static final double ODOM_Y_TICKS_PER_INCH = OTOS_Y_TICKS_PER_INCH;

    /*
     * Feedforward parameters - START WITH THESE VALUES
     */
    public static double kV = 1.0 / rpmToVelocity(MAX_RPM);
    public static double kA = 0;
    public static double kStatic = 0;

    /*
     * Motion constraints - CONSERVATIVE VALUES FOR TESTING
     * Increase these after calibration is complete
     */
    public static double MAX_VEL = 30;        // Start conservative
    public static double MAX_ACCEL = 30;      // Start conservative
    public static double MAX_ANG_VEL = Math.toRadians(90);   // Start conservative
    public static double MAX_ANG_ACCEL = Math.toRadians(90); // Start conservative

    // Utility functions
    public static double encoderTicksToInches(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    public static double rpmToVelocity(double rpm) {
        return rpm * GEAR_RATIO * 2 * Math.PI * WHEEL_RADIUS / 60.0;
    }

    public static double getMotorVelocityF(double ticksPerSecond) {
        return 32767 / ticksPerSecond;
    }
}