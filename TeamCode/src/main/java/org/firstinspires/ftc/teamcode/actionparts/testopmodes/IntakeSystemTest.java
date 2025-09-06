package org.firstinspires.ftc.teamcode.actionparts.testopmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.actionparts.Intakesystem;

@TeleOp(name="Intake System Test", group="Test")
public class IntakeSystemTest extends LinearOpMode {

    private Intakesystem intakeSystem;

    @Override
    public void runOpMode() {
        // Map the hardware motor (make sure the config name matches DS/RC config)
        DcMotor intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");

        // Create intake system
        intakeSystem = new Intakesystem(intakeMotor);

        // Optionally call initialize (if you add reset/brake logic later)
        intakeSystem.initialize();

        telemetry.addLine("Ready to run Intake Test");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // --- Controls ---
            if (gamepad1.a) {
                intakeSystem.startInTake(); // intake forward
            } else if (gamepad1.b) {
                intakeSystem.startReverseInTake(); // intake reverse
            } else if (gamepad1.x) {
                intakeSystem.stopInTake(); // stop intake
            }

            // --- Telemetry ---
            telemetry.addData("Motor Power", intakeMotor.getPower());
            telemetry.update();
        }
    }
}
