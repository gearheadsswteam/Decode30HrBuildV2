package org.firstinspires.ftc.teamcode.roadrunner;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

/**
 * Mecanum Drive class for Roadrunner 1.0.1
 * Integrates mecanum drive motors with SparkFun OTOS localizer
 */
public class MecanumDrive {

    // Drive constants - adjust these for your robot
    public static class Params {
        // Motor power limits
        public double maxWheelVel = 50.0;        // inches per second
        public double minProfileAccel = -30.0;   // inches per second squared
        public double maxProfileAccel = 50.0;    // inches per second squared

        // Drive constraints
        public double maxAngVel = Math.PI;       // radians per second
        public double maxAngAccel = Math.PI;     // radians per second squared

        // Track width and wheel base (adjust for your robot)
        public double trackWidth = 10.4;         // inches between left and right wheels
        public double wheelBase = 10.125;          // inches between front and back wheels

        // Wheel radius
        public double wheelRadius = 2.045;        // inches (96mm goBILDA mecanum wheels)

        // Motor ticks per revolution (for goBILDA 312 RPM motor)
        public double ticksPerRev = 145.1;
    }

    public final Params params;

    // Motors
    private final DcMotor leftFront, leftBack, rightBack, rightFront;

    // Localizer
    private final SparkFunOTOSLocalizer localizer;

    // IMU (backup/supplementary)
    private final IMU imu;

    /**
     * Constructor for MecanumDrive
     */

    public MecanumDrive(HardwareMap hardwareMap, Pose2d pose) {
        this.params = new Params();

        // Initialize motors
        leftFront = hardwareMap.get(DcMotor.class, "fl");
        leftBack = hardwareMap.get(DcMotor.class, "bl");
        rightBack = hardwareMap.get(DcMotor.class, "br");
        rightFront = hardwareMap.get(DcMotor.class, "fr");

        // Set initial directions (may need adjustment)
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.FORWARD);

        // Set brake mode
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Initialize IMU
        imu = hardwareMap.get(IMU.class, "gyro");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        imu.initialize(parameters);

        // Initialize localizer
        localizer = new SparkFunOTOSLocalizer(hardwareMap, pose);
    }

    /**
     * Set drive power using mecanum drive kinematics
     */
    public void setDrivePowers(PoseVelocity2d powers) {
        double x = powers.linearVel.x;
        double y = powers.linearVel.y;
        double rx = powers.angVel;

        // Mecanum drive calculations
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;

        // Set motor powers
        leftFront.setPower(frontLeftPower);
        leftBack.setPower(backLeftPower);
        rightFront.setPower(frontRightPower);
        rightBack.setPower(backRightPower);
    }

    public void setDrivePowers(double frontLeftPower, double backLeftPower, double frontRightPower, double backRightPower){
        // Set motor powers
        leftFront.setPower(frontLeftPower);
        leftBack.setPower(backLeftPower);
        rightFront.setPower(frontRightPower);
        rightBack.setPower(backRightPower);
    }

    /**
     * Update localizer and get current pose
     */
    public Pose2d updatePoseEstimate() {
        return localizer.update();
    }

    /**
     * Get current pose without updating
     */
    public Pose2d getPose() {
        return localizer.getCurrentPose();
    }

    /**
     * Reset pose to specified position
     */
    public void setPose(Pose2d pose) {
        localizer.resetPose(pose);
    }

    /**
     * Create action to drive to a specific pose
     */
    public Action driveTo(Pose2d targetPose) {
        return new DriveToPositionAction(targetPose);
    }

    /**
     * Create action to strafe to position while maintaining current heading
     */
    public Action strafeTo(Vector2d endPosition) {
        return new DriveToPositionAction(new Pose2d(endPosition, getPose().heading));
    }

    /**
     * Create action to turn to specific heading while maintaining position
     */
    public Action turnTo(double endHeading) {
        return new DriveToPositionAction(new Pose2d(getPose().position, endHeading));
    }

    /**
     * Create action to drive straight to position with specific heading
     */
    public Action strafeToLinearHeading(Vector2d endPosition, double endHeading) {
        return new DriveToPositionAction(new Pose2d(endPosition, endHeading));
    }

    /**
     * Action class for driving to a specific position
     */
    private class DriveToPositionAction implements Action {
        private final Pose2d targetPose;
        private final double positionTolerance = 1.0; // inches
        private final double headingTolerance = Math.toRadians(2); // degrees

        public DriveToPositionAction(Pose2d targetPose) {
            this.targetPose = targetPose;
        }

        @Override
        public boolean run(@androidx.annotation.NonNull com.acmerobotics.dashboard.telemetry.TelemetryPacket packet) {
            // Update pose
            Pose2d currentPose = updatePoseEstimate();

            // Calculate errors
            double xError = targetPose.position.x - currentPose.position.x;
            double yError = targetPose.position.y - currentPose.position.y;
            double headingError = targetPose.heading.toDouble() - currentPose.heading.toDouble();

            // Normalize heading error
            while (headingError > Math.PI) headingError -= 2 * Math.PI;
            while (headingError < -Math.PI) headingError += 2 * Math.PI;

            // Check if we've reached the target
            double positionError = Math.sqrt(xError * xError + yError * yError);
            if (positionError < positionTolerance && Math.abs(headingError) < headingTolerance) {
                setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));
                return false; // Action complete
            }

            // Simple proportional control
            double kP_trans = 0.8;
            double kP_rot = 1.0;

            // Calculate drive powers
            double x = xError * kP_trans;
            double y = yError * kP_trans;
            double rx = headingError * kP_rot;

            // Limit maximum speeds
            x = Math.max(-0.8, Math.min(0.8, x));
            y = Math.max(-0.8, Math.min(0.8, y));
            rx = Math.max(-0.8, Math.min(0.8, rx));

            // Set drive powers
            setDrivePowers(new PoseVelocity2d(new Vector2d(x, y), rx));

            // Add telemetry
            packet.put("Target X", targetPose.position.x);
            packet.put("Target Y", targetPose.position.y);
            packet.put("Target Heading", Math.toDegrees(targetPose.heading.toDouble()));
            packet.put("Current X", currentPose.position.x);
            packet.put("Current Y", currentPose.position.y);
            packet.put("Current Heading", Math.toDegrees(currentPose.heading.toDouble()));
            packet.put("Position Error", positionError);
            packet.put("Heading Error", Math.toDegrees(headingError));

            return true; // Continue action
        }
    }
}