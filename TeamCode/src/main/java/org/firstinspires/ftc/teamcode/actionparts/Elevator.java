package org.firstinspires.ftc.teamcode.actionparts;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Elevator {

    private DcMotor leftMotor;
    private DcMotor rightMotor;

    // Tuning constants
    public static final double POWER_UP   = 0.8;
    public static final double POWER_DOWN = -0.5;

    public Elevator(HardwareMap hardwareMap) {
        leftMotor  = hardwareMap.get(DcMotor.class, "elevatorLeft");
        rightMotor = hardwareMap.get(DcMotor.class, "elevatorRight");

        // Reset encoders
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // One motor is flipped mechanically
        leftMotor.setDirection(DcMotor.Direction.FORWARD);
        rightMotor.setDirection(DcMotor.Direction.REVERSE);

        // Use encoders for control
        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Hold position when stopped
        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    /** Manual control */
    public void setPower(double power) {
        leftMotor.setPower(power);
        rightMotor.setPower(power);
    }

    /** Go to a target encoder position */
    public void setTargetPosition(int ticks, double power) {
        leftMotor.setTargetPosition(ticks);
        rightMotor.setTargetPosition(ticks);

        leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftMotor.setPower(power);
        rightMotor.setPower(power);
    }

    /** Check if elevator is busy moving */
    public boolean isBusy() {
        return leftMotor.isBusy() || rightMotor.isBusy();
    }

    /** Stop motion and hold */
    public void stop() {
        leftMotor.setPower(0);
        rightMotor.setPower(0);
    }

    /** Get average encoder position (for telemetry) */
    public int getCurrentPosition() {
        return (leftMotor.getCurrentPosition() + rightMotor.getCurrentPosition()) / 2;
    }
}
