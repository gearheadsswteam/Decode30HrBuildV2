package org.firstinspires.ftc.teamcode.actionparts;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

public class FlywheelShooterWithRPMControl {

    // --- TUNE THESE ---
    public static final double TICKS_PER_REV = 537.7; // change to your motor/gearbox
    public static final double READY_BAND_RPM = 50;   // ± band for "at speed"
    public static final double RECOVERY_BAND_RPM = 75;
    public static final double FEED_DELAY_MS = 120;
    public static final double MIN_SPINUP_MS = 250;

    // PIDF @ 12V (conservative starting point)
    private static final double kP = 0.0008;
    private static final double kI = 0.0001;
    private static final double kD = 0.0000;
    private static final double kF_base = 0.11; // scaled by 12/voltage each loop

    private final DcMotorEx flyA, flyB;
    private final VoltageSensor battery;
    private final ElapsedTime sinceLastShot = new ElapsedTime();

    private double targetRpm = 0;
    private double lastSetF = -1;

    public FlywheelShooterWithRPMControl(HardwareMap hw, String motorAName, String motorBName) {
        flyA = hw.get(DcMotorEx.class, motorAName);
        flyB = hw.get(DcMotorEx.class, motorBName);

        // Directions: adjust so the contact surfaces both move "forward" along the ball path.
        // Typical dual-side shooter needs one reversed.
        // If your balls curve or speed fights each other, flip one direction.
        flyA.setDirection(DcMotorSimple.Direction.FORWARD);
        flyB.setDirection(DcMotorSimple.Direction.REVERSE);

        for (DcMotorEx m : new DcMotorEx[]{flyA, flyB}) {
            m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            m.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }

        battery = hw.voltageSensor.iterator().next();
        applyVoltageCompensatedPIDF();
        sinceLastShot.reset();
    }

    /** Call every loop to keep F scaled with battery voltage. */
    public void applyVoltageCompensatedPIDF() {
        double v = Math.max(10.0, battery.getVoltage());
        double kF = kF_base * (12.0 / v);
        if (Math.abs(kF - lastSetF) > 1e-4) {
            flyA.setVelocityPIDFCoefficients(kP, kI, kD, kF);
            flyB.setVelocityPIDFCoefficients(kP, kI, kD, kF);
            lastSetF = kF;
        }
    }

    /** Set desired flywheel speed in RPM. Use 0 to stop. */
    public void setTargetRpm(double rpm) {
        targetRpm = Math.max(0, rpm);
        double tps = rpmToTps(targetRpm);
        flyA.setVelocity(tps);
        flyB.setVelocity(tps);
    }

    /** Average RPM of both motors. */
    public double getCurrentRpm() {
        double rpmA = tpsToRpm(flyA.getVelocity());
        double rpmB = tpsToRpm(flyB.getVelocity());
        return 0.5 * (rpmA + rpmB);
    }

    /** Within band and minimum spin-up time elapsed. */
    public boolean isReadyToFire() {
        if (targetRpm <= 0) return false;
        if (sinceLastShot.milliseconds() < MIN_SPINUP_MS) return false;
        return Math.abs(getCurrentRpm() - targetRpm) <= READY_BAND_RPM;
    }

    /** Stricter check after a shot. */
    public boolean hasRecoveredSinceShot() {
        if (targetRpm <= 0) return false;
        return Math.abs(getCurrentRpm() - targetRpm) <= RECOVERY_BAND_RPM;
    }

    /** Call right after feeding one ball. */
    public void markShot() {
        sinceLastShot.reset();
    }

    public void stop() {
        setTargetRpm(0);
        flyA.setPower(0);
        flyB.setPower(0);
    }

    private static double rpmToTps(double rpm) { return rpm * TICKS_PER_REV / 60.0; }
    private static double tpsToRpm(double tps) { return tps * 60.0 / TICKS_PER_REV; }
}