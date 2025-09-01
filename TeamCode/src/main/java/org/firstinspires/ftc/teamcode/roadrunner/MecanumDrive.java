package org.firstinspires.ftc.teamcode.roadrunner;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.HolonomicController;
import com.acmerobotics.roadrunner.MecanumKinematics;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.MotorFeedforward;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Pose2dDual;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.PoseVelocity2dDual;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.ProfileParams;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.TimeTrajectory;
import com.acmerobotics.roadrunner.TimeTurn;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TrajectoryBuilderParams;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.TurnConstraints;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import java.util.Arrays;

/**
 * Road Runner 1.0.1 Mecanum Drive
 * - SparkFun OTOS localizer for pose
 * - IMU for heading
 * - Provides actionBuilder() for trajectories
 */
public final class MecanumDrive {
    // Motors
    private final DcMotorEx fl, fr, bl, br;

    // Localizer
    private final SparkFunOTOSLocalizer localizer;

    private final MecanumKinematics kinematics;


    // IMU
    private final IMU imu;

    // Feedforward (basic defaults — tune later)
    private final MotorFeedforward feedforward = new MotorFeedforward(0.0, 1.0, 0.0);

    // Controller (tune gains for your robot)
    private final HolonomicController controller =
            new HolonomicController(
                    2.0, 2.0, 4.0,   // positional gains (axial, lateral, heading)
                    0.0, 0.0, 0.0);  // velocity gains


    // Motor power limits
    public double maxWheelVel = 50.0;        // inches per second
    public double minProfileAccel = -30.0;   // inches per second squared
    public double maxProfileAccel = 50.0;    // inches per second squared

    // Drive constraints
    public double maxAngVel = Math.PI;       // radians per second
    public double maxAngAccel = Math.PI;     // radians per second squared

    // Track width and wheel base (adjust for your robot)
    public double trackWidth = 16.0;         // inches between left and right wheels
    public double wheelBase = 16.0;          // inches between front and back wheels

    // Wheel radius
    public double wheelRadius = 1.89;        // inches (96mm goBILDA mecanum wheels)

    // Motor ticks per revolution (for goBILDA 312 RPM motor)
    public double ticksPerRev = 537.7;



    public MecanumDrive(HardwareMap hardwareMap, Pose2d startPose) {
        fl = hardwareMap.get(DcMotorEx.class, "fl");
        fr = hardwareMap.get(DcMotorEx.class, "fr");
        bl = hardwareMap.get(DcMotorEx.class, "bl");
        br = hardwareMap.get(DcMotorEx.class, "br");

        fl.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        // Reverse left side if needed
        fl.setDirection(DcMotorSimple.Direction.REVERSE);
        bl.setDirection(DcMotorSimple.Direction.REVERSE);

        // IMU
        imu = hardwareMap.get(IMU.class, "gyro");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        imu.initialize(parameters);

        // Localizer
        localizer = new SparkFunOTOSLocalizer(hardwareMap, startPose);

        this.kinematics = new MecanumKinematics(trackWidth, wheelBase);

    }



    // ---------------- Drive Power APIs ----------------

    /** Road Runner style: set drive powers from PoseVelocity2d */
    public void setDrivePowers(PoseVelocity2d vel) {
        double x = vel.linearVel.x;
        double y = vel.linearVel.y;
        double rx = vel.angVel;

        double denom = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double flPow = (y + x + rx) / denom;
        double blPow = (y - x + rx) / denom;
        double frPow = (y - x - rx) / denom;
        double brPow = (y + x - rx) / denom;

        fl.setPower(flPow);
        bl.setPower(blPow);
        fr.setPower(frPow);
        br.setPower(brPow);
    }

    /** Direct motor control: raw powers (for TeleOp or test) */
    public void setDrivePowers(double flPower, double blPower, double frPower, double brPower) {
        fl.setPower(flPower);
        bl.setPower(blPower);
        fr.setPower(frPower);
        br.setPower(brPower);
    }

    // ---------------- Pose Estimation ----------------

    public Pose2d updatePoseEstimate() {
        return localizer.update();
    }

    public Pose2d getPose() {
        return localizer.getCurrentPose();
    }

    public void setPose(Pose2d pose) {
        localizer.resetPose(pose);
    }

    // ---------------- RR Factories ----------------
    private Action turnAsync(TimeTurn turn) {
        final double[] start = { -1 };

        return packet -> {
            double now = System.nanoTime() / 1e9;
            double t;
            if (start[0] < 0) {
                start[0] = now;
                t = 0;
            } else {
                t = now - start[0];
            }

            if (t >= turn.duration) {
                setDrivePowers(0, 0, 0, 0);
                return false;
            }

            // targetPose is Dual, as required
            Pose2dDual<Time> targetPose = turn.get(t);

            // actualPose is plain Pose2d
            Pose2d currentPose = updatePoseEstimate();

            // if you don’t have velocity estimates, just pass zero
            PoseVelocity2d currentVel = localizer.getPoseVelocity();

            // compute with (Dual, Pose2d, PoseVelocity2d)
            PoseVelocity2dDual<Time> commandDual =
                    controller.compute(targetPose, currentPose, currentVel);

            // extract plain velocity for motors
            PoseVelocity2d command = commandDual.value();

            setDrivePowers(command);

            packet.put("Target Heading", Math.toDegrees(targetPose.value().heading.toDouble()));
            packet.put("Current Heading", Math.toDegrees(currentPose.heading.toDouble()));
            return true;
        };
    }

    private Action followTrajectoryAsync(TimeTrajectory traj) {
        final double[] start = { -1 };

        return packet -> {
            double now = System.nanoTime() / 1e9;
            double t;
            if (start[0] < 0) {
                start[0] = now;
                t = 0;
            } else {
                t = now - start[0];
            }

            if (t >= traj.duration) {
                setDrivePowers(0, 0, 0, 0);
                return false;
            }

            Pose2dDual<Time> targetPose = traj.get(t);
            Pose2d currentPose = updatePoseEstimate();

            // substitute with real velocity if you can estimate it
            PoseVelocity2d currentVel = localizer.getPoseVelocity();

            PoseVelocity2dDual<Time> commandDual =
                    controller.compute(targetPose, currentPose, currentVel);

            PoseVelocity2d command = commandDual.value();

            setDrivePowers(command);

            packet.put("Target X", targetPose.value().position.x);
            packet.put("Target Y", targetPose.value().position.y);
            packet.put("Current X", currentPose.position.x);
            packet.put("Current Y", currentPose.position.y);
            return true;
        };
    }


    /** Entry point for Road Runner trajectory building */
    public TrajectoryActionBuilder actionBuilder(Pose2d startPose) {
        return new TrajectoryActionBuilder(
                this::turnAsync,
                this::followTrajectoryAsync,
                new TrajectoryBuilderParams(
                        1e-6,
                        new ProfileParams(0.25, 0.25, 1e-2)
                ),
                startPose,
                0.0,
                new TurnConstraints(maxAngVel, -maxAngAccel, maxAngAccel),
                new MinVelConstraint(Arrays.asList(
                        new TranslationalVelConstraint(maxWheelVel)
                )),
                new ProfileAccelConstraint(minProfileAccel, maxProfileAccel)
        );
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
