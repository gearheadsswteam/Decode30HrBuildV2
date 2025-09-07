package org.firstinspires.ftc.teamcode.actionparts.testopmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.actionparts.FlywheelShooterWithRPMControl;

@TeleOp(name="Flywheel PIDF Demo (Step RPM)", group="Shooter")
public class FlywheelTeleopDemo extends LinearOpMode {

    private static final int STEP_SIZE = 50;       // RPM step size
    private static final int MAX_RPM = 5000;       // upper clamp (tune for your wheel/motor)
    private static final int MIN_RPM = 0;          // lower clamp

    private FlywheelShooterWithRPMControl shooter;
    private Servo feeder;
    private static final double FEED_CLOSED = 0.05;
    private static final double FEED_OPEN   = 0.35;

    private double target = 0;
    private boolean dpadUpPrev = false;
    private boolean dpadDownPrev = false;

    @Override
    public void runOpMode() throws InterruptedException {
        shooter = new FlywheelShooterWithRPMControl(hardwareMap, "flyA", "flyB");
        feeder  = hardwareMap.get(Servo.class, "feeder");
        feeder.setPosition(FEED_CLOSED);

        FtcDashboard dash = FtcDashboard.getInstance();

        waitForStart();

        while (opModeIsActive()) {
            // Step control with dpad
            if (gamepad1.dpad_up && !dpadUpPrev) {
                target = Math.min(target + STEP_SIZE, MAX_RPM);
            }
            if (gamepad1.dpad_down && !dpadDownPrev) {
                target = Math.max(target - STEP_SIZE, MIN_RPM);
            }
            dpadUpPrev = gamepad1.dpad_up;
            dpadDownPrev = gamepad1.dpad_down;

            // Stop with X
            if (gamepad1.x) target = 0;

            shooter.applyVoltageCompensatedPIDF();
            shooter.setTargetRpm(target);

            boolean ready = shooter.isReadyToFire();

            // Fire ball when ready
            if (gamepad1.right_bumper && ready) {
                feeder.setPosition(FEED_OPEN);
                sleep((long) FlywheelShooterWithRPMControl.FEED_DELAY_MS);
                feeder.setPosition(FEED_CLOSED);
                shooter.markShot();

                // Wait for recovery before next shot
                while (opModeIsActive() && !gamepad1.right_bumper && !shooter.hasRecoveredSinceShot()) {
                    shooter.applyVoltageCompensatedPIDF();

                    telemetry.addData("Recovering", "RPM=%.0f / %.0f", shooter.getCurrentRpm(), target);
                    telemetry.update();

                    TelemetryPacket pkt = new TelemetryPacket();
                    pkt.put("targetRPM", target);
                    pkt.put("currentRPM", shooter.getCurrentRpm());
                    pkt.put("ready", false);
                    dash.sendTelemetryPacket(pkt);
                    idle();
                }
            }

            // Telemetry
            telemetry.addData("Target RPM", "%.0f", target);
            telemetry.addData("Current RPM", "%.0f", shooter.getCurrentRpm());
            telemetry.addData("Ready", shooter.isReadyToFire());
            telemetry.update();

            TelemetryPacket pkt = new TelemetryPacket();
            pkt.put("targetRPM", target);
            pkt.put("currentRPM", shooter.getCurrentRpm());
            pkt.put("ready", shooter.isReadyToFire());
            dash.sendTelemetryPacket(pkt);

            idle();
        }

        shooter.stop();
    }
}