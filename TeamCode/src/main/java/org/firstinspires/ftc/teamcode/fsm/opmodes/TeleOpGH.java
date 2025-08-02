
package org.firstinspires.ftc.teamcode.fsm.opmodes;

//opmodes;

import static org.firstinspires.ftc.teamcode.ValueStorage.redMultiplier;
import static java.lang.Math.PI;
import static java.lang.Math.atan2;
import static java.lang.Math.pow;
import static java.lang.Math.sin;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.ValueStorage;
import org.firstinspires.ftc.teamcode.fsm.hardware.GHRobot;
import org.firstinspires.ftc.teamcode.fsm.subsystems.*;
import org.firstinspires.ftc.teamcode.fsm.statemachine.RobotStateMachine;
import org.firstinspires.ftc.teamcode.fsm.statemachine.StateBuilder;
import org.firstinspires.ftc.teamcode.fsm.input.RisingEdgeDetector;
import org.firstinspires.ftc.teamcode.teleop.statemachine.GHRobotStates;
import org.firstinspires.ftc.teamcode.teleop.statemachine.RobotState;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "GH TeleOp")
public class TeleOpGH extends LinearOpMode {
    DcMotorEx fr;
    DcMotorEx fl;
    DcMotorEx br;
    DcMotorEx bl;

    BNO055IMU gyro;

    double initialHeading = ValueStorage.lastPose.heading.toDouble() - redMultiplier * PI / 2;
    double robotHeading;
    double joystickAngle;
    double joystickMagnitude;
    double turn;

    GHRobot robot = new GHRobot();
    IntakeSubsystem intake;
    DeliverySubsystem delivery;
    RisingEdgeDetector edge = new RisingEdgeDetector();
    RobotStateMachine fsm = null;
    boolean fsmTriggered = false;

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        intake = new IntakeSubsystem(robot);
        delivery = new DeliverySubsystem(robot);

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

        fsm = StateBuilder.buildLoopingFSM(telemetry, intake, delivery);


        waitForStart();

        while (opModeIsActive()) {
            // Advance FSM one state each time "a" is pressed
            if (edge.isPressed(gamepad1, "a")) {
                fsm.triggerNext();
            }

            // Optionally: manual overrides
            if (edge.isPressed(gamepad1, "x")) {

                intake.intake(0.5);
            }

            if (edge.isPressed(gamepad1, "y")) {
                intake.stop();
            }

            fsm.update();

            telemetry.addData("FSM Current State", fsm.getCurrentStateName());
            telemetry.update();
        }
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
