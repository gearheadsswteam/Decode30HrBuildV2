package org.firstinspires.ftc.teamcode.actionparts.testopmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.actionparts.Claw;

@TeleOp(name="Claw Test (Toggle)", group="Test")
public class ClawTest extends LinearOpMode {

    private Claw claw;
    private boolean lastButtonState = false;

    @Override
    public void runOpMode() {
        claw = new Claw(hardwareMap, "clawServo");

        telemetry.addLine("Ready to run Claw Test (Toggle)");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // --- Toggle with A button ---
            boolean currentButtonState = gamepad1.a;
            if (currentButtonState && !lastButtonState) {
                claw.toggle();
            }
            lastButtonState = currentButtonState;

            // --- Telemetry ---
            telemetry.addData("Claw State", claw.isOpen() ? "OPEN" : "CLOSED");
            telemetry.addData("Servo Position", claw.getPosition());
            telemetry.update();
        }
    }
}
