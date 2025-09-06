package org.firstinspires.ftc.teamcode.autonomous;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot.LogoFacingDirection;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot.UsbFacingDirection;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

/**
 * Minimal, self-contained API for 4-wheel mecanum autonomous moves using encoders for linear motion
 * and an IMU for heading control. Drop this file into your TeamCode module (you can also split into
 * separate files if you prefer). Includes:
 *   - forward/backward by inches
 *   - strafe right/left by inches (with lateral correction multiplier)
 *   - turn right/left by degrees (IMU-based PID)
 *
 * Usage: see demo OpMode at the bottom of this file.
 */
public class MecanumAutoAPI {
    private final LinearOpMode op;
    private final DcMotorEx fl, fr, bl, br;
    private final BNO055IMU imu;

    // === TUNE THESE FOR YOUR ROBOT ===
    public static class Params {
        // === Robot geometry (fill from your measurements) ===
        public double trackWidth = 10.4;        // inches between left & right wheels (user)
        public double wheelBase  = 10.125;      // inches between front & back wheels (user)

        // Wheel & encoder
        public double wheelDiameterIn = 2.0 * 2.045; // 4.09 in (user wheelRadius = 2.045")
        public double gearReduction   = 1.0;         // output (wheel) / motor
        public double ticksPerMotorRev = 383.6; // goBILDA 5202 Yellow Jacket 13.7:1 (435 RPM)      // USER-SUPPLIED; verify your motor/encoder CPR
        // Note: Many FTC motors are 383.6 or 537.6. If your moves seem off, confirm this value.

        // Strafing usually needs more ticks per inch to overcome scrub
        public double lateralMultiplier = 1.10;       // start ~1.10–1.20, empirically tune

        // Motion limits
        public double maxLinearPower = 0.7;
        public double maxTurnPower   = 0.6;

        // IMU turning PID (degrees)
        public double kP = 0.015;   // power per degree error
        public double kI = 0.0;     // add only if steady-state error
        public double kD = 0.0025;  // damp overshoot
        public double minTurnPower = 0.12; // overcome static friction
        public double turnOnTargetDeg = 1.0;
        public double turnSettledTimeS = 0.15;

        // Safety
        public double stallTimeoutS = 6.0;
    }

    private final Params params;
    private final double COUNTS_PER_INCH;

    /** Public accessor for CPI (Counts Per Inch). */
    public double countsPerInch() { return COUNTS_PER_INCH; }

    /** Print key calibration parameters to DS telemetry. Call during init. */
    public void logCalibrationTelemetry() {
        op.telemetry.addLine("=== MecanumAutoAPI Cal ===");
        op.telemetry.addData("trackWidth (in)", params.trackWidth);
        op.telemetry.addData("wheelBase (in)", params.wheelBase);
        op.telemetry.addData("wheelDiameter (in)", params.wheelDiameterIn);
        op.telemetry.addData("ticksPerRev", params.ticksPerMotorRev);
        op.telemetry.addData("Counts/ Inch (CPI)", "%.3f", COUNTS_PER_INCH);
        op.telemetry.addData("lateralMultiplier", params.lateralMultiplier);
        op.telemetry.update();
    }

    public MecanumAutoAPI(LinearOpMode opMode,
                          HardwareMap hw,
                          String flName, String frName, String blName, String brName,
                          RevHubOrientationOnRobot hubOrientation, // required for correct yaw
                          Params p) {
        this.op = opMode;
        this.params = (p == null) ? new Params() : p;

        fr = hw.get(DcMotorEx.class, "fr");
        fl = hw.get(DcMotorEx.class, "fl");
        br = hw.get(DcMotorEx.class, "br");
        bl = hw.get(DcMotorEx.class, "bl");
        imu = hw.get(BNO055IMU.class, "gyro");

        fr.setDirection(DcMotorSimple.Direction.REVERSE);
        br.setDirection(DcMotorSimple.Direction.REVERSE);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        imu.initialize(parameters);

        for (DcMotorEx m : new DcMotorEx[]{fl, fr, bl, br}) {
            m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            m.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        COUNTS_PER_INCH = (params.ticksPerMotorRev * params.gearReduction) / (params.wheelDiameterIn * Math.PI);
    }

    public static RevHubOrientationOnRobot defaultREVHubOrientation() {
        // EDIT: set these to match your Control Hub/Expansion Hub mounting
        return new RevHubOrientationOnRobot(LogoFacingDirection.RIGHT, UsbFacingDirection.FORWARD);
    }

    // === Public API ===
    public void forward(double inches, double power, double timeoutS) {
        linearMove(inches, inches, inches, inches, clamp(power, 0, params.maxLinearPower), timeoutS);
    }

    public void backward(double inches, double power, double timeoutS) {
        forward(-inches, power, timeoutS);
    }

    public void strafeRight(double inches, double power, double timeoutS) {
        double t = inches * params.lateralMultiplier;
        // mecanum strafe: + - - + (for FR,FL,BL,BR depending on directions). With our directions:
        linearMove(+t, -t, -t, +t, clamp(power, 0, params.maxLinearPower), timeoutS);
    }

    public void strafeLeft(double inches, double power, double timeoutS) {
        strafeRight(-inches, power, timeoutS);
    }

    public void turnRight(double degrees, double timeoutS) { // CW negative yaw in IMU framework usually
        turnIMU(-Math.abs(degrees), timeoutS);
    }

    public void turnLeft(double degrees, double timeoutS) {
        turnIMU(+Math.abs(degrees), timeoutS);
    }

    public double getHeadingDeg() {
       return imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
    }

    // === Core helpers ===
    private void linearMove(double flIn, double frIn, double blIn, double brIn, double power, double timeoutS) {
        if (!op.opModeIsActive()) return;
        int flTarget = fl.getCurrentPosition() + (int) Math.round(flIn * COUNTS_PER_INCH);
        int frTarget = fr.getCurrentPosition() + (int) Math.round(frIn * COUNTS_PER_INCH);
        int blTarget = bl.getCurrentPosition() + (int) Math.round(blIn * COUNTS_PER_INCH);
        int brTarget = br.getCurrentPosition() + (int) Math.round(brIn * COUNTS_PER_INCH);

        setRunToPosition(flTarget, frTarget, blTarget, brTarget);
        setAllPower(power);

        ElapsedTime t = new ElapsedTime();
        t.reset();
        while (op.opModeIsActive() && t.seconds() < timeoutS && allBusy()) {
            op.telemetry.addData("Targets", "FL %d FR %d BL %d BR %d", flTarget, frTarget, blTarget, brTarget);
            op.telemetry.addData("Current", "FL %d FR %d BL %d BR %d", fl.getCurrentPosition(), fr.getCurrentPosition(), bl.getCurrentPosition(), br.getCurrentPosition());
            op.telemetry.update();
        }
        stopAndFloat();
    }

    private void turnIMU(double deltaDeg, double timeoutS) {
        if (!op.opModeIsActive()) return;
        // Use RUN_WITHOUT_ENCODER for smooth turning
        for (DcMotorEx m : new DcMotorEx[]{fl, fr, bl, br}) m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        final double start = getHeadingDeg();
        final double target = wrapDeg(start + deltaDeg);
        final ElapsedTime timer = new ElapsedTime();
        final ElapsedTime onTargetTimer = new ElapsedTime();
        timer.reset();
        onTargetTimer.reset();
        double prevError = angleDiffDeg(target, getHeadingDeg());
        double integral = 0;

        while (op.opModeIsActive() && timer.seconds() < timeoutS) {
            double heading = getHeadingDeg();
            double error = angleDiffDeg(target, heading); // shortest signed path (-180..+180)
            double dt = 0.02; // approx loop time (~50Hz)
            integral += error * dt;
            double deriv = (error - prevError) / dt;

            double cmd = params.kP * error + params.kI * integral + params.kD * deriv;
            cmd = clamp(cmd, -params.maxTurnPower, params.maxTurnPower);

            // Ensure we overcome static friction
            if (Math.abs(cmd) < params.minTurnPower) {
                cmd = Math.copySign(params.minTurnPower, cmd);
            }

            // Positive cmd -> CCW (left). With our motor directions, left turn is +power on left wheels negative on right.
            fl.setPower(+cmd);
            bl.setPower(+cmd);
            fr.setPower(-cmd);
            br.setPower(-cmd);

            boolean onTarget = Math.abs(error) <= params.turnOnTargetDeg;
            if (onTarget) {
                // require remaining within window for a bit to settle
                if (onTargetTimer.seconds() >= params.turnSettledTimeS) break;
            } else {
                onTargetTimer.reset();
            }

            prevError = error;
            op.telemetry.addData("Turn", "target %.1f, heading %.1f, err %.2f, cmd %.2f", target, heading, error, cmd);
            op.telemetry.update();
            op.sleep(20);
        }
        stopAndBrake();
        // Restore encoder mode
        for (DcMotorEx m : new DcMotorEx[]{fl, fr, bl, br}) m.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private void setRunToPosition(int flT, int frT, int blT, int brT) {
        fl.setTargetPosition(flT);
        fr.setTargetPosition(frT);
        bl.setTargetPosition(blT);
        br.setTargetPosition(brT);
        for (DcMotorEx m : new DcMotorEx[]{fl, fr, bl, br}) m.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    private boolean allBusy() {
        return fl.isBusy() && fr.isBusy() && bl.isBusy() && br.isBusy();
    }

    private void setAllPower(double p) {
        fl.setPower(Math.abs(p));
        fr.setPower(Math.abs(p));
        bl.setPower(Math.abs(p));
        br.setPower(Math.abs(p));
    }

    private void stopAndFloat() {
        for (DcMotorEx m : new DcMotorEx[]{fl, fr, bl, br}) m.setPower(0);
        // Keep RUN_USING_ENCODER after run-to-position completes
        for (DcMotorEx m : new DcMotorEx[]{fl, fr, bl, br}) m.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private void stopAndBrake() {
        for (DcMotorEx m : new DcMotorEx[]{fl, fr, bl, br}) m.setPower(0);
    }

    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }

    private static double wrapDeg(double a) {
        while (a <= -180) a += 360;
        while (a > 180) a -= 360;
        return a;
    }

    private static double angleDiffDeg(double target, double current) {
        double diff = wrapDeg(target - current);
        return diff;
    }
}
