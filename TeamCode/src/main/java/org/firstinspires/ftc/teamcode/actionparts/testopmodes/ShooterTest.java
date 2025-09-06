package org.firstinspires.ftc.teamcode.actionparts.testopmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.actionparts.Shooter;

@TeleOp(name="Shooter Test", group="Test")
public class ShooterTest extends LinearOpMode {

    private Shooter shooter;

    @Override
    public void runOpMode() {
        // Init shooter (make sure config names match DS/RC config)
        shooter = new Shooter(hardwareMap, "leftShooter", "rightShooter");

        telemetry.addLine("Ready to run Shooter Test");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Toggle with button A
            if (gamepad1.a) {
                shooter.start(); // full power
            } else if (gamepad1.b) {
                shooter.stop();
            }

            // Test variable power with right trigger
            if (gamepad1.right_trigger > 0.1) {
                shooter.start(gamepad1.right_trigger); // scale power
            }

            telemetry.addData("Shooter Running", shooter.isRunning());
            telemetry.update();
        }
    }
}
