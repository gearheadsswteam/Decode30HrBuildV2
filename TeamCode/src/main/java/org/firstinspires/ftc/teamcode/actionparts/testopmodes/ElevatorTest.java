package org.firstinspires.ftc.teamcode.actionparts.testopmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Elevator;

@TeleOp(name="Elevator Test", group="Test")
public class ElevatorTest extends LinearOpMode {

    private Elevator elevator;

    @Override
    public void runOpMode() {
        // Initialize subsystem
        elevator = new Elevator(hardwareMap);

        telemetry.addLine("Ready to run Elevator Test");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // --- Manual joystick control ---
            double power = -gamepad1.left_stick_y; // push stick up = elevator up
            elevator.setPower(power);

            // --- Preset levels with buttons ---
            if (gamepad1.a) {
                elevator.setTargetPosition(1000, 0.8); // Level 1
            }
            if (gamepad1.b) {
                elevator.setTargetPosition(2000, 0.8); // Level 2
            }
            if (gamepad1.y) {
                elevator.setTargetPosition(0, 0.6);    // Reset to bottom
            }

            // --- Emergency stop ---
            if (gamepad1.x) {
                elevator.stop();
            }

            // --- Telemetry ---
            telemetry.addData("Elevator Pos", elevator.getCurrentPosition());
            telemetry.addData("Elevator Busy", elevator.isBusy());
            telemetry.update();
        }
    }
}
