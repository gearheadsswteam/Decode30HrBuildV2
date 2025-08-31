package org.firstinspires.ftc.teamcode.roadrunner;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.Twist2dDual;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * SparkFun OTOS Localizer for Roadrunner 1.0.1
 * Provides pose estimation using SparkFun Optical Tracking Odometry Sensor
 */
public class SparkFunOTOSLocalizer {

    private final SparkFunOTOS otos;
    private Pose2d currentPose;

    /**
     * Constructor for SparkFun OTOS Localizer
     * @param hardwareMap The robot's hardware map
     * @param startPose The starting pose of the robot
     */
    public SparkFunOTOSLocalizer(HardwareMap hardwareMap, Pose2d startPose) {
        // Get OTOS from hardware map
        otos = hardwareMap.get(SparkFunOTOS.class, "otos");

        // Configure OTOS
        configureOTOS();

        // Set initial pose
        currentPose = startPose;
        setOTOSPose(startPose);
    }

    /**
     * Configure the OTOS sensor with appropriate settings
     */
    private void configureOTOS() {
        // Reset OTOS calibration
        otos.resetTracking();

        // The OTOS sensor works in inches and radians by default in FTC SDK
        // No need to set units - they're already correct

        // Set OTOS offset (adjust these values based on your robot's OTOS mounting position)
        // These values assume OTOS is mounted at center of robot
        SparkFunOTOS.Pose2D offset = new SparkFunOTOS.Pose2D(0, 0, 0);
        otos.setOffset(offset);

        // Set linear and angular scalars (start with 1.0, tune if needed)
        otos.setLinearScalar(1.0);
        otos.setAngularScalar(1.0);

        // Calibrate IMU (this may take a few seconds)
        otos.calibrateImu();

        // Reset position to origin
        otos.resetTracking();
    }

    /**
     * Update and get the current pose from OTOS sensor
     * @return The current pose of the robot
     */
    public Pose2d update() {
        // Get pose from OTOS
        SparkFunOTOS.Pose2D otosPose = otos.getPosition();

        // Convert to Roadrunner Pose2d
        currentPose = new Pose2d(
                otosPose.x,
                otosPose.y,
                otosPose.h
        );

        return currentPose;
    }

    /**
     * Set the pose in the OTOS sensor
     * @param pose The pose to set
     */
    private void setOTOSPose(Pose2d pose) {
        SparkFunOTOS.Pose2D otosPose = new SparkFunOTOS.Pose2D(
                pose.position.x,
                pose.position.y,
                pose.heading.toDouble()
        );
        otos.setPosition(otosPose);
    }

    /**
     * Reset the localizer position
     * @param pose The new pose to reset to
     */
    public void resetPose(Pose2d pose) {
        currentPose = pose;
        setOTOSPose(pose);
    }

    /**
     * Get the current pose without updating
     * @return The last known pose
     */
    public Pose2d getCurrentPose() {
        return currentPose;
    }

    /**
     * Get velocity from OTOS (if needed for advanced applications)
     * @return The current velocity
     */
    public Vector2d getVelocity() {
        SparkFunOTOS.Pose2D velocity = otos.getVelocity();
        return new Vector2d(velocity.x, velocity.y);
    }

    /**
     * Get pose velocity from OTOS
     * @return The current pose velocity (position + angular)
     */
    public PoseVelocity2d getPoseVelocity() {
        SparkFunOTOS.Pose2D velocity = otos.getVelocity();
        return new PoseVelocity2d(
                new Vector2d(velocity.x, velocity.y),
                velocity.h
        );
    }
}