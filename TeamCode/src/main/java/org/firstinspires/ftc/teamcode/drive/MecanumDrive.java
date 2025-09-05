package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.hardware.RoadRunnerOTOSLocalizer;

import java.util.Arrays;
import java.util.List;

@Config
public class MecanumDrive {

    // Motor objects
    private DcMotorEx leftFront, leftBack, rightBack, rightFront;
    private List<DcMotorEx> motors;

    // Sensors
    private IMU imu;
    private VoltageSensor voltageSensor;

    // OTOS localizer
    private RoadRunnerOTOSLocalizer localizer;

    public MecanumDrive(HardwareMap hardwareMap) {
        // Initialize motors
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");

        motors = Arrays.asList(leftFront, leftBack, rightBack, rightFront);

        // Set motor directions (adjust as needed for your robot)
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.FORWARD);

        // Set zero power behavior
        for (DcMotorEx motor : motors) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        // Initialize IMU
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        imu.initialize(parameters);

        voltageSensor = hardwareMap.voltageSensor.iterator().next();

        // Initialize OTOS localizer
        localizer = new RoadRunnerOTOSLocalizer(hardwareMap);
    }

    /**
     * Set drive powers using mecanum drive kinematics
     */
    public void setDrivePowers(PoseVelocity2d powers) {
        double x = powers.linearVel.x;
        double y = powers.linearVel.y;
        double rx = powers.angVel;

        // Mecanum drive calculations
        double leftFrontPower = x + y + rx;
        double leftBackPower = x - y + rx;
        double rightBackPower = x + y - rx;
        double rightFrontPower = x - y - rx;

        // Normalize powers if any exceed 1.0
        double max = Math.max(Math.abs(leftFrontPower), Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));
        max = Math.max(max, Math.abs(rightFrontPower));

        if (max > 1.0) {
            leftFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
            rightFrontPower /= max;
        }

        // Set motor powers
        leftFront.setPower(leftFrontPower);
        leftBack.setPower(leftBackPower);
        rightBack.setPower(rightBackPower);
        rightFront.setPower(rightFrontPower);
    }

    /**
     * Get current pose estimate from localizer
     */
    public Pose2d getPose() {
        return localizer.getPoseEstimate();
    }

    /**
     * Update localizer
     */
    public void updatePoseEstimate() {
        localizer.update();
    }

    /**
     * Set pose estimate
     */
    public void setPose(Pose2d pose) {
        localizer.setPoseEstimate(pose);
    }

    /**
     * Get current velocity estimate
     */
    public PoseVelocity2d getVelocity() {
        return localizer.getVelocityEstimate();
    }

    /**
     * Get IMU heading in radians
     */
    public double getHeading() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
    }

    /**
     * Reset IMU heading
     */
    public void resetHeading() {
        imu.resetYaw();
    }
}