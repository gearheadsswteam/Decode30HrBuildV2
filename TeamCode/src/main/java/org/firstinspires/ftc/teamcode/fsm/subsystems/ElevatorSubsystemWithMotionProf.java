package org.firstinspires.ftc.teamcode.fsm.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Two-motor elevator subsystem using trapezoidal motion profiling and PID control.
 * <p>
 * This class coordinates two elevator motors with a custom PID controller and trapezoidal
 * motion profiling to enable smooth and controlled motion between predefined levels.
 */
public class ElevatorSubsystemWithMotionProf {

    /**
     * Predefined elevator levels in encoder ticks.
     */
    public enum Level {
        LOW(0), MID(1000), HIGH(2000);

        public final int positionTicks;

        Level(int ticks) {
            this.positionTicks = ticks;
        }
    }

    private final DcMotorEx leftMotor;
    private final DcMotorEx rightMotor;

    // PID coefficients
    private final double kP = 0.01;
    private final double kI = 0.0;
    private final double kD = 0.002;

    // Motion profiling parameters
    private final double maxSpeed = 1.0;        // Maximum motor power (0.0 to 1.0)
    private final int accelDistance = 300;      // Distance to apply acceleration/deceleration

    private int targetPosition = 0;
    private double previousError = 0;
    private double integral = 0;

    /**
     * Constructs and initializes the elevator subsystem using two motors.
     *
     * @param hardwareMap the hardware map to retrieve motor references
     */
    public ElevatorSubsystemWithMotionProf(HardwareMap hardwareMap) {
        leftMotor = hardwareMap.get(DcMotorEx.class, "elevator_left");
        rightMotor = hardwareMap.get(DcMotorEx.class, "elevator_right");

        leftMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        leftMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        rightMotor.setDirection(DcMotorEx.Direction.REVERSE);

        leftMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
    }

    /**
     * Sets the target level to move the elevator to.
     *
     * @param level the desired elevator level
     */
    public void goTo(Level level) {
        targetPosition = level.positionTicks;
    }

    /**
     * Updates the elevator control loop.
     * Should be called periodically in the main loop.
     */
    public void update() {
        int currentPosition = (leftMotor.getCurrentPosition() + rightMotor.getCurrentPosition()) / 2;
        int distanceToGo = targetPosition - currentPosition;
        int absDistance = Math.abs(distanceToGo);
        int direction = Integer.signum(distanceToGo);

        // Motion profiling: compute limited speed based on proximity to target
        double profileSpeed;
        if (absDistance < accelDistance) {
            // Deceleration zone
            profileSpeed = maxSpeed * Math.sqrt((double) absDistance / accelDistance);
        } else {
            // Constant speed zone
            profileSpeed = maxSpeed;
        }

        // PID control
        double error = distanceToGo;
        integral += error;
        double derivative = error - previousError;
        previousError = error;

        double pidOutput = kP * error + kI * integral + kD * derivative;

        // Clamp power using profile speed
        double finalPower = direction * Math.min(Math.abs(pidOutput), profileSpeed);

        leftMotor.setPower(finalPower);
        rightMotor.setPower(finalPower);
    }

    /**
     * Checks if the elevator has reached its target position.
     *
     * @return true if current position is within tolerance of target
     */
    public boolean isAtTarget() {
        int currentPosition = (leftMotor.getCurrentPosition() + rightMotor.getCurrentPosition()) / 2;
        return Math.abs(targetPosition - currentPosition) < 20;
    }

    /**
     * Immediately stops both elevator motors.
     */
    public void stop() {
        leftMotor.setPower(0);
        rightMotor.setPower(0);
    }

    /**
     * Returns the average encoder position of both elevator motors.
     *
     * @return current elevator position in ticks
     */
    public int getCurrentPosition() {
        return (leftMotor.getCurrentPosition() + rightMotor.getCurrentPosition()) / 2;
    }

    /**
     * Returns the current target elevator position.
     *
     * @return target position in encoder ticks
     */
    public int getTargetPosition() {
        return targetPosition;
    }
}
