package org.firstinspires.ftc.teamcode.drive;

public class DriveConstants {

    /*
     * These are motor constants that should be listed online for your motors.
     */
    public static final double TICKS_PER_REV = 537.7; // GoBILDA 312 RPM Yellow Jacket
    public static final double MAX_RPM = 312;

    /*
     * Set RUN_USING_ENCODER to true to enable built-in hub velocity control using drive encoders.
     * Set this flag to false if drive encoders are not present and an alternative localization
     * method is in use (e.g. optical odometry).
     */
    public static final boolean RUN_USING_ENCODER = false; // Using OTOS instead

    public static final double MOTOR_VELO_PID_P = 0.0;
    public static final double MOTOR_VELO_PID_I = 0.0;
    public static final double MOTOR_VELO_PID_D = 0.0;
    public static final double MOTOR_VELO_PID_F = 0.0;

    /*
     * These are physical constants that can be determined from your robot (including the track
     * width; it will be tune empirically later although a rough estimate is important).
     */
    public static double WHEEL_RADIUS = 1.88976; // in
    public static double GEAR_RATIO = 1; // output (wheel) speed / input (motor) speed
    public static double TRACK_WIDTH = 15.0; // in

    /*
     * These are the feedforward parameters used to model the drive motor behavior. If you are using
     * the built-in velocity PID, *these values are fine as is*. However, if you do not have drive
     * encoders or have elected not to use them for velocity control, these values should be empirically
     * tuned.
     */
    public static double kV = 1.0 / rpmToVelocity(MAX_RPM);
    public static double kA = 0;
    public static double kStatic = 0;

    /*
     * These values are used to generate the trajectories for you robot. To ensure proper operation,
     * the constraints should never exceed ~80% of the robot's actual capabilities. While Road Runner
     * is designed to enable faster autonomous motion, it is a good idea for testing to start small
     * and gradually increase them later after everything is working. All distance units are inches.
     */
    public static double MAX_VEL = 50;
    public static double MAX_ACCEL = 50;
    public static double MAX_ANG_VEL = Math.toRadians(180);
    public static double MAX_ANG_ACCEL = Math.toRadians(180);

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