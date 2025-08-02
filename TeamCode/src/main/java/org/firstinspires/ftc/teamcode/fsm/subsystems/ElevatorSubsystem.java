package org.firstinspires.ftc.teamcode.fsm.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Subsystem for controlling a lift/elevator using PIDF to reach preset heights.
 */
public class ElevatorSubsystem {
    private final DcMotorEx elevatorMotor;

    // Preset target positions in encoder ticks
    private final int LOW = 0;
    private final int MID = 1000;
    private final int HIGH = 2000;

    private int targetPosition = 0;

    public ElevatorSubsystem(HardwareMap hardwareMap) {
        elevatorMotor = hardwareMap.get(DcMotorEx.class, "elevator");
        elevatorMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        elevatorMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        elevatorMotor.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(5.0, 0, 0, 0));

        elevatorMotor.setTargetPosition(0);
        elevatorMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        elevatorMotor.setPower(0.6);
    }

    public void goToLow() {
        targetPosition = LOW;
        moveToTarget();
    }

    public void goToMid() {
        targetPosition = MID;
        moveToTarget();
    }

    public void goToHigh() {
        targetPosition = HIGH;
        moveToTarget();
    }

    private void moveToTarget() {
        elevatorMotor.setTargetPosition(targetPosition);
        elevatorMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        elevatorMotor.setPower(0.6);
    }

    public boolean isAtTarget() {
        return Math.abs(elevatorMotor.getCurrentPosition() - targetPosition) < 20;
    }

    public void stop() {
        elevatorMotor.setPower(0);
    }
}