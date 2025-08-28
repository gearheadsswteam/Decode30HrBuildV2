package org.firstinspires.ftc.teamcode.drive;

import androidx.annotation.NonNull;
import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.localization.OpticalFlowLocalizer;
import org.firstinspires.ftc.teamcode.sensors.SparkFunOpticalFlow;
import com.qualcomm.hardware.rev.RevIMU;

import java.util.Arrays;
import java.util.List;

import static org.firstinspires.ftc.teamcode.drive.DriveConstants.*;

public class SampleMecanumDrive extends MecanumDrive {
    private final List<DcMotorEx> motors;

    public SampleMecanumDrive(HardwareMap hardwareMap) {
        super(kV, kA, kStatic, TRACK_WIDTH, WHEEL_BASE, LATERAL_MULTIPLIER);

        DcMotorEx leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        DcMotorEx leftRear = hardwareMap.get(DcMotorEx.class, "leftRear");
        DcMotorEx rightRear = hardwareMap.get(DcMotorEx.class, "rightRear");
        DcMotorEx rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");

        motors = Arrays.asList(leftFront, leftRear, rightRear, rightFront);

        for (DcMotorEx m : motors) {
            m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightRear.setDirection(DcMotor.Direction.REVERSE);

        SparkFunOpticalFlow ofX = new SparkFunOpticalFlow(hardwareMap, "of_x", 0x2A);
        SparkFunOpticalFlow ofY = new SparkFunOpticalFlow(hardwareMap, "of_y", 0x2B);
        RevIMU imu = hardwareMap.get(RevIMU.class, "imu");
        setLocalizer(new OpticalFlowLocalizer(hardwareMap, ofX, ofY, imu));
    }

    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        return Arrays.asList(0.0, 0.0, 0.0, 0.0);
    }

    @Override
    public void setMotorPowers(double v0, double v1, double v2, double v3) {
        motors.get(0).setPower(v0);
        motors.get(1).setPower(v1);
        motors.get(2).setPower(v2);
        motors.get(3).setPower(v3);
    }

    @Override
    public double getRawExternalHeading() {
        return 0.0;
    }
}
