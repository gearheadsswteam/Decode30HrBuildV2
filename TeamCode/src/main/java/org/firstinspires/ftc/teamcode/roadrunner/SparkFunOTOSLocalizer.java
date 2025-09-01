package org.firstinspires.ftc.teamcode.roadrunner;

import static java.lang.Math.PI;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * SparkFun OTOS Localizer for Roadrunner 1.0.1
 * FIXED: Rotates OTOS pose by -90 degrees to align with robot-forward frame
 */
public class SparkFunOTOSLocalizer {

    public final SparkFunOTOS otos;
    private Pose2d currentPose;

    public SparkFunOTOSLocalizer(HardwareMap hardwareMap, Pose2d startPose) {
        otos = hardwareMap.get(SparkFunOTOS.class, "otos");
        configureOTOS();
        currentPose = startPose;
        setOTOSPose(startPose);
    }

    private void configureOTOS() {
        otos.resetTracking();

        // Sensor offset (tune as needed based on mounting location)
        SparkFunOTOS.Pose2D offset = new SparkFunOTOS.Pose2D(-2.16, -3.38, PI / 2);
        otos.setOffset(offset);

        // Set scalars (tune if necessary)
        otos.setLinearScalar(1.0);
        otos.setAngularScalar(1.0);

        // Calibrate IMU
        otos.calibrateImu(255, true);

        otos.resetTracking();
    }

    public Pose2d update() {
        SparkFunOTOS.Pose2D otosPose = otos.getPosition();

        // Rotate OTOS coordinates by -90° to match robot frame
        double cos = Math.cos(-PI / 2);
        double sin = Math.sin(-PI / 2);

        double rotatedX = otosPose.x * cos - otosPose.y * sin;
        double rotatedY = otosPose.x * sin + otosPose.y * cos;
        double rotatedHeading = otosPose.h - PI / 2;

        // Normalize heading to [-π, π]
        rotatedHeading = Math.atan2(Math.sin(rotatedHeading), Math.cos(rotatedHeading));

        currentPose = new Pose2d(rotatedX, rotatedY, rotatedHeading);
        return currentPose;
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

        return new PoseVelocity2d(new Vector2d(vx, vy), velocity.h);
    }
}
