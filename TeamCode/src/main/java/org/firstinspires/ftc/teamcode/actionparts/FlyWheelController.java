package org.firstinspires.ftc.teamcode.actionparts;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.Range;

public class FlyWheelController {
    private DcMotorEx leftSpinnerMotor, rightSpinnerMotor;
    private VoltageSensor batteryVoltageSensor;

    private double targetRPM = 0;   // commanded setpoint
    private double kP = 0.0005;     // tune this!
    private static final double NOMINAL_VOLTAGE = 13.0;

    // ticks per revolution (adjust to your motor!)
    private static final double TICKS_PER_REV = 537.7;

    public FlyWheelController(DcMotorEx left, DcMotorEx right, HardwareMap hwMap) {
        this.leftSpinnerMotor = left;
        this.rightSpinnerMotor = right;

        leftSpinnerMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        rightSpinnerMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        leftSpinnerMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightSpinnerMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        leftSpinnerMotor.setDirection(DcMotor.Direction.REVERSE);
        rightSpinnerMotor.setDirection(DcMotor.Direction.FORWARD);


        // use the first available battery sensor
        batteryVoltageSensor = hwMap.voltageSensor.iterator().next();
    }

    public void setkP(double newKp) {
        this.kP = newKp;
    }

    /** API: set target RPM */
    public void spinnersOn(double rpm) {
        this.targetRPM = rpm;
    }

    /** API: stop flywheel */
    public void spinnersOff() {
        this.targetRPM = 0;
        leftSpinnerMotor.setPower(0);
        rightSpinnerMotor.setPower(0);
    }

    /** Call this every loop (TeleOp or Autonomous) */
    public void update() {
        if (targetRPM <= 0) return; // off, do nothing

        double currentVoltage = batteryVoltageSensor.getVoltage();
        double voltageComp = NOMINAL_VOLTAGE / currentVoltage;

        // Read current RPM
        double leftRPM = ticksPerSecToRPM(leftSpinnerMotor.getVelocity());
        double rightRPM = ticksPerSecToRPM(rightSpinnerMotor.getVelocity());

        // Error
        double leftError = targetRPM - leftRPM;
        double rightError = targetRPM - rightRPM;

        // Compute new powers with P control + voltage compensation
        double leftPower = Range.clip((leftSpinnerMotor.getPower() + kP * leftError) * voltageComp, -1.0, 1.0);
        double rightPower = Range.clip((rightSpinnerMotor.getPower() + kP * rightError) * voltageComp, -1.0, 1.0);

        leftSpinnerMotor.setPower(leftPower);
        rightSpinnerMotor.setPower(rightPower);
    }

    private double ticksPerSecToRPM(double ticksPerSec) {
        return (ticksPerSec / TICKS_PER_REV) * 60.0;
    }
}
