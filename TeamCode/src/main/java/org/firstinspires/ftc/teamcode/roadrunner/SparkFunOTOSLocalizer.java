package org.firstinspires.ftc.teamcode.roadrunner;

import static java.lang.Math.PI;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.HardwareMap;

// ✅ ADDED FOR IMU
import com.qualcomm.hardware.bosch.BNO055IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

/**
 * SparkFun OTOS Localizer for Roadrunner 1.0.1
 * ✅ Fallback to Control Hub IMU for heading when OTOS heading is too noisy
 */
public class SparkFunOTOSLocalizer {

    public final SparkFunOTOS otos;
    public final BNO055IMU imu; // ✅ ADDED
    private Pose2d currentPose;

    public SparkFunOTOSLocalizer(HardwareMap hardwareMap, Pose2d startPose) {
        otos = hardwareMap.get(SparkFunOTOS.class, "otos");
        imu = hardwareMap.get(BNO055IMU.class, "gyro"); // ✅ ADDED

        configureOTOS();
        configureIMU(); // ✅ ADDED

        currentPose = startPose;
        setOTOSPose(startPose);
    }

    private void configureOTOS() {
        otos.resetTracking();

        // ✅ You may update this offset based on your mounting
        SparkFunOTOS.Pose2D offset = new SparkFunOTOS.Pose2D(-2.16, -3.38, PI / 2);
        otos.setOffset(offset);

        otos.setLinearScalar(48/47.84);
        otos.setAngularScalar(0.0); // ✅ CHANGED: Ignore OTOS heading because we use IMU instead

        otos.calibrateImu(255, true);
        otos.resetTracking();
    }

    private void configureIMU() { // ✅ ADDED
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;

        imu.initialize(parameters);

        while (!imu.isGyroCalibrated()) {
            // Wait until IMU is calibrated
        }
    }

    public Pose2d update() {
        SparkFunOTOS.Pose2D otosPose = otos.getPosition();

        // ✅ Rotate OTOS coordinates by -90 degrees
        double cos = Math.cos(-PI / 2);
        double sin = Math.sin(-PI / 2);

        double rotatedX = otosPose.x * cos - otosPose.y * sin;
        double rotatedY = otosPose.x * sin + otosPose.y * cos;

        double imuHeading = getImuHeading(); // ✅ ADDED

        currentPose = new Pose2d(rotatedX, rotatedY, imuHeading);
        return currentPose;
    }

    private double getImuHeading() { // ✅ ADDED
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
        return angles.firstAngle;
    }

    private void setOTOSPose(Pose2d pose) {
        SparkFunOTOS.Pose2D otosPose = new SparkFunOTOS.Pose2D(
                pose.position.x,
                pose.position.y,
                pose.heading.toDouble()
        );
        otos.setPosition(otosPose);
    }

    public void resetPose(Pose2d pose) {
        currentPose = pose;
        setOTOSPose(pose);
    }

    public Pose2d getCurrentPose() {
        return currentPose;
    }

    public Vector2d getVelocity() {
        SparkFunOTOS.Pose2D velocity = otos.getVelocity();

        double cos = Math.cos(-PI / 2);
        double sin = Math.sin(-PI / 2);

        double vx = velocity.x * cos - velocity.y * sin;
        double vy = velocity.x * sin + velocity.y * cos;

        return new Vector2d(vx, vy);
    }

    public PoseVelocity2d getPoseVelocity() {
        SparkFunOTOS.Pose2D velocity = otos.getVelocity();

        double cos = Math.cos(-PI / 2);
        double sin = Math.sin(-PI / 2);

        double vx = velocity.x * cos - velocity.y * sin;
        double vy = velocity.x * sin + velocity.y * cos;

        return new PoseVelocity2d(new Vector2d(vx, vy), 0.0); // ✅ CHANGED: angular velocity = 0
    }
}
