package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class OpticalOdometryLocalizer {
    private SparkFunOTOS odomSensor;

    private Pose2d poseEstimate = new Pose2d(0, 0, 0);
    private Pose2d lastPose = new Pose2d(0, 0, 0);
    private double lastUpdateTime = 0;

    public OpticalOdometryLocalizer(HardwareMap hardwareMap) {
        // Initialize your SparkFun optical sensors here
        // This is a simplified version - you'll need to adapt based on your specific sensors
        odomSensor = hardwareMap.get(SparkFunOTOS.class, "sparkfun_odom");
    }

    public Pose2d getPoseEstimate() {
        return poseEstimate;
    }

    public void setPoseEstimate(Pose2d pose2d) {
        poseEstimate = pose2d;
        // Reset sensors to match this pose
        odomSensor.resetPosition();
    }

    public void update() {
        // Read from optical sensors and convert to pose
        double x = odomSensor.getXPosition() / DriveConstants.ODOM_X_TICKS_PER_INCH;
        double y = odomSensor.getYPosition() / DriveConstants.ODOM_Y_TICKS_PER_INCH;
        double heading = Math.toRadians(odomSensor.getHeading() / 100.0); // Assuming heading in centidegrees

        poseEstimate = new Pose2d(x, y, heading);
        lastUpdateTime = System.nanoTime() / 1e9;
    }
}