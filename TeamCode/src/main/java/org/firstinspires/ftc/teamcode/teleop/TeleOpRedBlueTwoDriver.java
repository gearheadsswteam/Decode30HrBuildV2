package org.firstinspires.ftc.teamcode.teleop;

import static org.firstinspires.ftc.teamcode.ValueStorage.redMultiplier;
import static java.lang.Math.PI;
import static java.lang.Math.atan2;
import static java.lang.Math.pow;
import static java.lang.Math.sin;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.GHRobot;
import org.firstinspires.ftc.teamcode.ValueStorage;
//import org.firstinspires.ftc.teamcode.teleop.statemachine.GHRobotState;
import org.firstinspires.ftc.teamcode.teleop.statemachine.GHRobotStates;
import org.firstinspires.ftc.teamcode.teleop.statemachine.RobotState;
import org.firstinspires.ftc.teamcode.teleop.statemachine.RobotStateMachine;

@TeleOp(name = "TeleOpTwoController", group = "TeleOp")
public class TeleOpRedBlueTwoDriver extends LinearOpMode {
    DcMotorEx fr;
    DcMotorEx fl;
    DcMotorEx br;
    DcMotorEx bl;

    BNO055IMU gyro;

    private RobotState currentState = GHRobotStates.IDLE;

    double initialHeading = ValueStorage.lastPose.heading.toDouble() - redMultiplier * PI / 2;
    double robotHeading;
    double joystickAngle;
    double joystickMagnitude;
    double turn;

    GHRobot robot = new GHRobot();
    RisingEdgeDetector detector;
    RobotStateMachine stateMachine = new RobotStateMachine(robot);

    @Override
    public void runOpMode() {
        fr = hardwareMap.get(DcMotorEx.class, "fr");
        fl = hardwareMap.get(DcMotorEx.class, "fl");
        br = hardwareMap.get(DcMotorEx.class, "br");
        bl = hardwareMap.get(DcMotorEx.class, "bl");
        gyro = hardwareMap.get(BNO055IMU.class, "gyro");
        fr.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        fl.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        fr.setDirection(Direction.REVERSE);
        br.setDirection(Direction.REVERSE);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        gyro.initialize(parameters);

        detector = new RisingEdgeDetector();

        waitForStart();

        //Stop motors to prevent accidental starts after autonomous
        fr.setPower(0);
        fl.setPower(0);
        br.setPower(0);
        bl.setPower(0);


        while (opModeIsActive() && !isStopRequested()) {
            runStateMachineLogic();
            telemetry.addData("Current State", currentState);
            telemetry.update();

            moveRobot();
        }
    }


    private void runStateMachineLogic() {
        if (detector.isRisingEdge("a", gamepad1.a)) {
            stateMachine.requestAdvance();
        }

        // Automatically transition when timer completes
        boolean didAdvance = stateMachine.update();

        telemetry.addData("Current State", stateMachine.getCurrentState());
        telemetry.addData("Elapsed Time", stateMachine.getElapsedTime());
        telemetry.addData("Required Time", stateMachine.getRequiredDuration());
        telemetry.addData("Ready to transition", stateMachine.isTransitionComplete());
        telemetry.addData("Pending", didAdvance ? "✅ Transitioned" : "⏳ Waiting...");
        telemetry.update();
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
}