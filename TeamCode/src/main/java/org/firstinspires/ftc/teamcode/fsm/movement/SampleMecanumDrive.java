package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.roadrunner.drive.DriveConstants;
import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import java.util.List;

import org.firstinspires.ftc.teamcode.drive.rrconfigs.DriveConstantsRR;
import org.firstinspires.ftc.teamcode.drive.rrconfigs.StandardTrackingWheelLocalizer;

public class SampleMecanumDrive extends MecanumDrive {

    private final DcMotorEx leftFront, leftRear, rightRear, rightFront;
    private final List<DcMotorEx> motors;
    private final VoltageSensor batteryVoltageSensor;

    public SampleMecanumDrive(HardwareMap hardwareMap) {
        super(DriveConstantsRR.kV, DriveConstantsRR.kA, DriveConstantsRR.kStatic, DriveConstantsRR.TRACK_WIDTH);

        leftFront = hardwareMap.get(DcMotorEx.class, "fl");
        leftRear = hardwareMap.get(DcMotorEx.class, "bl");
        rightRear = hardwareMap.get(DcMotorEx.class, "br");
        rightFront = hardwareMap.get(DcMotorEx.class, "fr");

        motors = List.of(leftFront, leftRear, rightRear, rightFront);

        for (DcMotorEx motor : motors) {
            motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            motor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        }

        leftFront.setDirection(DcMotorEx.Direction.REVERSE);
        leftRear.setDirection(DcMotorEx.Direction.REVERSE);

        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();

        setLocalizer(new StandardTrackingWheelLocalizer(hardwareMap));
    }

    @Override
    public List<Double> getWheelPositions() {
        return List.of(
                (double) leftFront.getCurrentPosition(),
                (double) leftRear.getCurrentPosition(),
                (double) rightRear.getCurrentPosition(),
                (double)
