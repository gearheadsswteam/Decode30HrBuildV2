package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.actionparts.Intakesystem;
import org.firstinspires.ftc.teamcode.robot.GearheadsMecanumRobotRR;

public class AbstractAuto extends LinearOpMode {
    private GearheadsMecanumRobotRR robot;   // Use gearheads robot hardware

    //Different action systems used by the Robot
    private Intakesystem intakesystem;


    @Override
    public void runOpMode() throws InterruptedException {
        initOpMode();
    }

    /**
     * Initializes the auto opmode
     */
    private void initOpMode() {

        telemetry.addData("Status", "Waiting to start...");
        telemetry.update();
        robot = new GearheadsMecanumRobotRR(this);
        // Wait for the game to start (driver presses PLAY)
        //Need this as first step else we get 5 point penalty
        waitForStart();

        telemetry.addData("Status", "Started");
        telemetry.update();

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.initTeleOp(hardwareMap);
        intakesystem = robot.intakesystem;

        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }
}
