package org.firstinspires.ftc.teamcode.actionparts;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

public class FlywheelShooterWithRPMControl {

    private final DcMotorEx flyA, flyB;
    private final VoltageSensor battery;
    private final ElapsedTime sinceLastShot = new ElapsedTime();

    private double targetRpm = 0;
    private double lastSetF = -1;

    public FlywheelShooterWithRPMControl(HardwareMap hw, String motorAName, String motorBName) {
        flyA = hw.get(DcMotorEx.class, motorAName);
        flyB = hw.get(DcMotorEx.class, motorBName);

        flyA.setDirection(ShooterTuning.MOTOR_A_REVERSED
                ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        flyB.setDirection(ShooterTuning.MOTOR_B_REVERSED
                ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);

        for (DcMotorEx m : new DcMotorEx[]{flyA, flyB}) {
            m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            m.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }

        battery = hw.voltageSensor.iterator().next();
        applyVoltageCompensatedPIDF();
        sinceLastShot.reset();
    }

    /** Call every loop to keep F scaled with battery voltage and refresh PID. */
    public void applyVoltageCompensatedPIDF() {
        double v = Math.max(10.0, battery.getVoltage()); // guard against very low readings
        double kF = ShooterTuning.kF_base * (12.0 / v);
        if (Math.abs(kF - lastSetF) > 1e-4) {
            flyA.setVelocityPIDFCoefficients(ShooterTuning.kP, ShooterTuning.kI, ShooterTuning.kD, kF);
            flyB.setVelocityPIDFCoefficients(ShooterTuning.kP, ShooterTuning.kI, ShooterTuning.kD, kF);
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

    /** True when within ready band and minimum spin-up time elapsed. */
    public boolean isReadyToFire() {
        if (targetRpm <= 0) return false;
        if (sinceLastShot.milliseconds() < ShooterTuning.MIN_SPINUP_MS) return false;
        return Math.abs(getCurrentRpm() - targetRpm) <= ShooterTuning.READY_BAND_RPM;
    }

    /** Stricter check after a shot before allowing next feed. */
    public boolean hasRecoveredSinceShot() {
        if (targetRpm <= 0) return false;
        return Math.abs(getCurrentRpm() - targetRpm) <= ShooterTuning.RECOVERY_BAND_RPM;
    }

    /** Call right after you feed one ball. */
    public void markShot() {
        sinceLastShot.reset();
    }

    public void stop() {
        setTargetRpm(0);
        flyA.setPower(0);
        flyB.setPower(0);
    }

    private static double rpmToTps(double rpm) { return rpm * ShooterTuning.TICKS_PER_REV / 60.0; }
    private static double tpsToRpm(double tps) { return tps * 60.0 / ShooterTuning.TICKS_PER_REV; }
}
