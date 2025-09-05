package org.firstinspires.ftc.teamcode.hardware;

import static java.lang.Math.PI;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.hardware.Otos;
import org.firstinspires.ftc.teamcode.hardware.OtosLocalizer;
import org.firstinspires.ftc.teamcode.movement.Pose;
import org.firstinspires.ftc.teamcode.movement.Twist;

/**
 * Bridge between OTOS localizer and RoadRunner's expected interface
 */
public class RoadRunnerOTOSLocalizer {
    private OtosLocalizer otosLocalizer;
    private Otos otos;

    public RoadRunnerOTOSLocalizer(HardwareMap hardwareMap) {
        // Initialize OTOS sensor
        otos = hardwareMap.get(Otos.class, "otos");

        // Initialize OTOS and calibrate
        if (!otos.begin()) {
            throw new RuntimeException("OTOS sensor not found!");
        }

        // Calibrate IMU (this takes ~600ms)
        otos.calibrateImu();
        otos.setOffset(new Pose(-2.16, -3.38, -PI/2));

        // Set units to inches and degrees for FTC compatibility
        otos.setLinearUnit(org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.INCH);
        otos.setAngularUnit(org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES);

        // Create localizer with 10ms update delay (100Hz max)
        otosLocalizer = new OtosLocalizer(otos,0.01);
    }

    /**
     * Get current pose as RoadRunner Pose2d
     */
    public Pose2d getPoseEstimate() {
        Pose pos = otosLocalizer.pos;
        return new Pose2d(pos.x, pos.y, pos.h);
    }

    /**
     * Set pose estimate as RoadRunner Pose2d
     */
    public void setPoseEstimate(Pose2d pose2d) {
        Pose pose = new Pose(pose2d.position.x, pose2d.position.y, Math.toDegrees(pose2d.heading.toDouble()));
        otosLocalizer.setPose(pose);
    }

    /**
     * Get current velocity as RoadRunner PoseVelocity2d
     */
    public PoseVelocity2d getVelocityEstimate() {
        Twist velocity = otosLocalizer.vel(System.nanoTime() / 1e9);
        return new PoseVelocity2d(
                new Vector2d(velocity.x, velocity.y),
                Math.toRadians(velocity.h)
        );
    }

    /**
     * Update localizer - call this in your main loop
     */
    public void update() {
        otosLocalizer.update(System.nanoTime() / 1e9);
    }
}