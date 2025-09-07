package org.firstinspires.ftc.teamcode.teleop;


import static java.lang.Math.PI;
import static java.lang.Math.atan2;
import static java.lang.Math.pow;
import static java.lang.Math.sin;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.ValueStorage;
import org.firstinspires.ftc.teamcode.actionparts.Claw;
import org.firstinspires.ftc.teamcode.actionparts.Elevator;
import org.firstinspires.ftc.teamcode.actionparts.Intakesystem;
import org.firstinspires.ftc.teamcode.robot.GearheadsMecanumRobotRR;

@TeleOp(name = "TeleOpTwoController", group = "TeleOp")
public class TeleOpRedBlueTwoDriverFSM extends LinearOpMode {
    DcMotorEx fr;
    DcMotorEx fl;
    DcMotorEx br;
    DcMotorEx bl;

    BNO055IMU gyro;

    double initialHeading = ValueStorage.lastPose.heading.toDouble();
    double robotHeading;
    double joystickAngle;
    double joystickMagnitude;
    double turn;

    RisingEdgeDetector detector;

    /* Declare OpMode members. */
    private GearheadsMecanumRobotRR robot;   // Use gearheads robot hardware

    // Subsystems (ADDED)
    private Intakesystem intakesystem; // will be mapped by motor name "intake"
    private Claw claw; // will be mapped by servo name "claw"
    private Elevator elevator; // uses elevatorLeft/elevatorRight internally

    // ADDED: Reusable Timed State Machine instance (subsystems only; driving stays manual)
    private final TimedStateMachine fsm = new TimedStateMachine();

    @Override
    public void runOpMode() {

        fr = hardwareMap.get(DcMotorEx.class, "fr");
        fl = hardwareMap.get(DcMotorEx.class, "fl");
        br = hardwareMap.get(DcMotorEx.class, "br");
        bl = hardwareMap.get(DcMotorEx.class, "bl");
        gyro = hardwareMap.get(BNO055IMU.class, "gyro");
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setDirection(DcMotorSimple.Direction.REVERSE);
        br.setDirection(DcMotorSimple.Direction.REVERSE);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        gyro.initialize(parameters);

        detector = new RisingEdgeDetector();


        //Stop motors to prevent accidental starts after autonomous
        fr.setPower(0);
        fl.setPower(0);
        br.setPower(0);
        bl.setPower(0);

        initOpMode();
        setupFSM();
        boolean prevX = false;

        while (opModeIsActive() && !isStopRequested()) {
            moveRobot();

            // ADDED: FSM controls — X to start, Back to cancel
            boolean xPressed = gamepad1.x && !prevX; // rising edge

            if (gamepad1.back) {
                fsm.cancel();
                stopSubsystems();
            }

            if (xPressed) {
                fsm.onTrigger();   // <- exactly matches your rules
            }
            prevX = gamepad1.x;

            fsm.update();

            telemetry.addData("FSM", fsm.isRunning() ? "RUN" : "IDLE");
            telemetry.addData("Step", "%d / %d", fsm.currentIndex(), fsm.size());
            telemetry.addData("Awaiting Advance", fsm.isAwaitingAdvance());
            telemetry.update();
        }
    }

    // ADDED: One method to define the 4 states you specified
    private void setupFSM() {
        final int ELEV_POS_1_TICKS = 250; // TODO: tune
        final int ELEV_POS_2_TICKS = 1100; // TODO: tune
        fsm.clear()
// (1) Intake ON for 5 seconds
                .add(new TimedStateMachine.Step()
                        .onStart(() -> {
                            if (intakesystem != null) intakesystem.startInTake();
                        })
                        .maxDurationMS(5000)
                        .onStop(() -> {
                            if (intakesystem != null) intakesystem.stopInTake();
                        })
                )
// (2) Elevator -> position 2, Intake STOP, up to 3 seconds (ends early if reached)
                .add(new TimedStateMachine.Step()
                        .onStart(() -> {
                            if (intakesystem != null) intakesystem.stopInTake();
                            if (elevator != null)
                                elevator.setTargetPosition(ELEV_POS_2_TICKS, Elevator.POWER_UP);
                        })
                        .doneWhen(() -> elevator != null && !elevator.isBusy())
                        .maxDurationMS(3000)
                        .onStop(() -> {
                            if (elevator != null) elevator.stop();
                        })
                )
// (3) Claw CLOSE takes 1 second
                .add(new TimedStateMachine.Step()
                        .onStart(() -> {
                            if (claw != null) claw.close();
                        })
                        .maxDurationMS(1000)
                )
// (4) Elevator -> position 1 and Claw OPEN, up to 2 seconds (ends early if reached)
                .add(new TimedStateMachine.Step()
                        .onStart(() -> {
                            if (claw != null) claw.open();
                            if (elevator != null)
                                elevator.setTargetPosition(ELEV_POS_1_TICKS, Elevator.POWER_DOWN);
                        })
                        .doneWhen(() -> elevator != null && !elevator.isBusy())
                        .maxDurationMS(2000)
                        .onStop(() -> {
                            if (elevator != null) elevator.stop();
                        })
                );
    }


    private void moveRobot() {
        if (gamepad1.ps) {
            initialHeading -= robotHeading;
        }
        robotHeading = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle + initialHeading;
        joystickAngle = atan2(-gamepad1.left_stick_x, -gamepad1.left_stick_y);
        joystickMagnitude = pow(gamepad1.left_stick_x, 2) + pow(gamepad1.left_stick_y, 2);
        turn = gamepad1.right_stick_x * Math.abs(gamepad1.right_stick_x);

        if (gamepad1.right_trigger < 0.1) {
            fr.setPower(-Range.clip(joystickMagnitude * sin(PI / 4 + joystickAngle - robotHeading) - turn, -1, 1));
            fl.setPower(-Range.clip(joystickMagnitude * sin(PI / 4 - joystickAngle + robotHeading) + turn, -1, 1));
            br.setPower(-Range.clip(joystickMagnitude * sin(PI / 4 - joystickAngle + robotHeading) - turn, -1, 1));
            bl.setPower(-Range.clip(joystickMagnitude * sin(PI / 4 + joystickAngle - robotHeading) + turn, -1, 1));
        } else {//speed Damper if right trigger pressed.
            fr.setPower(-Range.clip(joystickMagnitude * sin(PI / 4 + joystickAngle - robotHeading) - turn, -1, 1) / 3);
            fl.setPower(-Range.clip(joystickMagnitude * sin(PI / 4 - joystickAngle + robotHeading) + turn, -1, 1) / 3);
            br.setPower(-Range.clip(joystickMagnitude * sin(PI / 4 - joystickAngle + robotHeading) - turn, -1, 1) / 3);
            bl.setPower(-Range.clip(joystickMagnitude * sin(PI / 4 + joystickAngle - robotHeading) + turn, -1, 1) / 3);
        }
    }

    /**
     * Operate intake system
     */
    private void operateIntake() {
        if (gamepad2.a) {
            intakesystem.startInTake();
        }
        if (gamepad2.b) {
            intakesystem.stopInTake();
        }
        if (gamepad2.y) {
            intakesystem.startReverseInTake();
        }
    }


    /**
     * Initialize the opmode
     */

    private void initOpMode() {
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
        //TODO Add Elevator
        //TODO add Claw


        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    // Helper to stop all subsystems when cancelling FSM
    private void stopSubsystems() {
        if (intakesystem != null) intakesystem.stopInTake();
        if (elevator != null) elevator.stop();
    }
}