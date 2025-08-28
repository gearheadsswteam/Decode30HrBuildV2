package org.firstinspires.ftc.teamcode.fsm.drive;

import androidx.annotation.NonNull;

// RR-specific imports
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.util.Encoder;

import java.util.Arrays;
import java.util.List;

public class SampleMecanumDrive extends MecanumDrive {

    private final DcMotor frontLeft, rearLeft, rearRight, frontRight;
    private final List<DcMotor> motors;

    private final Encoder leftEncoder, rightEncoder, frontEncoder;

    public MecanumDrive(HardwareMap hardwareMap) {
        super(DriveConstants.kV, DriveConstants.kA, DriveConstants.kStatic,
                DriveConstants.TRACK_WIDTH, DriveConstants.WHEEL_BASE, DriveConstants.LATERAL_MULTIPLIER);

        frontLeft = hardwareMap.get(DcMotor.class, "fl");
        rearLeft = hardwareMap.get(DcMotor.class, "bl");
        rearRight = hardwareMap.get(DcMotor.class, "br");
        frontRight = hardwareMap.get(DcMotor.class, "fr");

        motors = Arrays.asList(frontLeft, rearLeft, rearRight, frontRight);

        for (DcMotor motor : motors) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        rearLeft.setDirection(DcMotor.Direction.REVERSE);

        // Optional: Set up encoders for localization
        leftEncoder = new Encoder(hardwareMap.get(DcMotor.class, "bl"));
        rightEncoder = new Encoder(hardwareMap.get(DcMotor.class, "br"));
        frontEncoder = new Encoder(hardwareMap.get(DcMotor.class, "fr"));

        // Optional if using built-in localization:
        setLocalizer(new StandardTrackingWheelLocalizer(hardwareMap));
    }

    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        return Arrays.asList(
                (double) frontLeft.getCurrentPosition(),
                (double) rearLeft.getCurrentPosition(),
                (double) rearRight.getCurrentPosition(),
                (double) frontRight.getCurrentPosition()
        );
    }

    @NonNull
    @Override
    public List<Double> getWheelVelocities() {
        return Arrays.asList(
                (double) frontLeft.getVelocity(),
                (double) rearLeft.getVelocity(),
                (double) rearRight.getVelocity(),
                (double) frontRight.getVelocity()
        );
    }

    @Override
    public void setMotorPowers(double v, double v1, double v2, double v3) {
        frontLeft.setPower(v);
        rearLeft.setPower(v1);
        rearRight.setPower(v2);
        frontRight.setPower(v3);
    }

    @Override
    public double getRawExternalHeading() {
        // Override this if you're using an IMU
        return 0;
    }
}
