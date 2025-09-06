package org.firstinspires.ftc.teamcode.actionparts;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Shooter {

    private DcMotor leftMotor;
    private DcMotor rightMotor;

    // Default power (you can tune this for distance/speed)
    private static final double DEFAULT_POWER = 1.0;

    public Shooter(HardwareMap hardwareMap, String leftName, String rightName) {
        leftMotor  = hardwareMap.get(DcMotor.class, leftName);
        rightMotor = hardwareMap.get(DcMotor.class, rightName);

        // Ensure they spin in opposite directions
        leftMotor.setDirection(DcMotor.Direction.FORWARD);
        rightMotor.setDirection(DcMotor.Direction.REVERSE);

        // Motors stop when power = 0
        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    /** Start shooter at default power */
    public void start() {
        start(DEFAULT_POWER);
    }

    /** Start shooter with a given power */
    public void start(double power) {
        leftMotor.setPower(power);
        rightMotor.setPower(power);
    }

    /** Stop shooter */
    public void stop() {
        leftMotor.setPower(0);
        rightMotor.setPower(0);
    }

    /** Check if shooter is running */
    public boolean isRunning() {
        return Math.abs(leftMotor.getPower()) > 0.05 || Math.abs(rightMotor.getPower()) > 0.05;
    }
}
