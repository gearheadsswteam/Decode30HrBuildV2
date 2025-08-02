package org.firstinspires.ftc.teamcode.fsm.opmodes;

// Imports...
import static org.firstinspires.ftc.teamcode.ValueStorage.redMultiplier;
import static java.lang.Math.PI;
import static java.lang.Math.atan2;
import static java.lang.Math.pow;
import static java.lang.Math.sin;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.ValueStorage;
import org.firstinspires.ftc.teamcode.fsm.commands.*;
import org.firstinspires.ftc.teamcode.fsm.hardware.GHRobot;
import org.firstinspires.ftc.teamcode.fsm.input.RisingEdgeDetector;
import org.firstinspires.ftc.teamcode.fsm.statemachine.RobotStateMachine;
import org.firstinspires.ftc.teamcode.fsm.statemachine.StateBuilder;
import org.firstinspires.ftc.teamcode.fsm.subsystems.*;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "GH TeleOp")
public class TeleOpGH extends LinearOpMode {

    DcMotorEx fr, fl, br, bl;
    BNO055IMU gyro;

    double initialHeading = ValueStorage.lastPose.heading.toDouble() - redMultiplier * PI / 2;
    double robotHeading, joystickAngle, joystickMagnitude, turn;

    GHRobot robot = new GHRobot();
    IntakeSubsystem intake;
    DeliverySubsystem delivery;
    ElevatorSubsystem elevator;
    RisingEdgeDetector edge = new RisingEdgeDetector();
    RobotStateMachine fsm = null;
    RobotCommandRunner manualRunner = new RobotCommandRunner();

    @Override
    public void runOpMode() {
        // Init hardware
        robot.init(hardwareMap);
        intake = new IntakeSubsystem(robot);
        delivery = new DeliverySubsystem(robot);
        elevator = new ElevatorSubsystem(hardwareMap);

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

        // Build FSM
        fsm = StateBuilder.buildLoopingFSM(telemetry, intake, delivery);

        waitForStart();

        while (opModeIsActive()) {
            // FSM control: advance to next state on "a"
            if (edge.isPressed(gamepad1, "a")) {
                fsm.triggerNext();
            }

            // Run FSM
            fsm.update();

            // Manual command trigger: intake on "x"
            if (edge.isPressed(gamepad1, "x")) {
                manualRunner.schedule(new StateAwareTimedCommand(telemetry, "ManualIntake", 1500) {
                    @Override public void execute() {
                        intake.intake(0.6);
                    }
                    @Override public void end() {
                        intake.stop();
                        super.end();
                    }
                });
            }

            // Manual command trigger: delivery on "y"
            if (edge.isPressed(gamepad1, "y")) {
                manualRunner.schedule(new StateAwareTimedCommand(telemetry,"ManualDeliver", 1200) {
                    @Override public void execute() {
                        delivery.deliver(0.5);
                    }
                    @Override public void end() {
                        delivery.stop();
                        super.end();
                    }
                });
            }

            // Run manually scheduled commands
            manualRunner.update();

            telemetry.addData("FSM Current State", fsm.getCurrentStateName());
            telemetry.addData("Manual Commands Running", manualRunner.isBusy());
            telemetry.update();

            moveRobot();
            moveElevator();
        }
    }

    private void moveElevator(){
        if (edge.isPressed(gamepad1, "dpad_up")) {
            manualRunner.schedule(new ElevatorCommand(telemetry, elevator, ElevatorCommand.Level.HIGH));
        }
        if (edge.isPressed(gamepad1, "dpad_left")) {
            manualRunner.schedule(new ElevatorCommand(telemetry, elevator, ElevatorCommand.Level.MID));
        }
        if (edge.isPressed(gamepad1, "dpad_down")) {
            manualRunner.schedule(new ElevatorCommand(telemetry, elevator, ElevatorCommand.Level.LOW));
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

        double scale = (gamepad1.right_trigger < 0.1) ? 1.0 : 1.0 / 3;

        fr.setPower(-Range.clip(joystickMagnitude * sin(PI / 4 + joystickAngle - robotHeading) - turn, -1, 1) * scale);
        fl.setPower(-Range.clip(joystickMagnitude * sin(PI / 4 - joystickAngle + robotHeading) + turn, -1, 1) * scale);
        br.setPower(-Range.clip(joystickMagnitude * sin(PI / 4 - joystickAngle + robotHeading) - turn, -1, 1) * scale);
        bl.setPower(-Range.clip(joystickMagnitude * sin(PI / 4 + joystickAngle - robotHeading) + turn, -1, 1) * scale);
    }
}
