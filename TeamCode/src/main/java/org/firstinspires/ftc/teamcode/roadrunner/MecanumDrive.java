package org.firstinspires.ftc.teamcode.roadrunner;

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
    private HolonomicController controller =
            new HolonomicController(
                    0.5, 0.5, 0.5,   // positional gains (axial, lateral, heading)
                    0.15, 0.15, 0.25);  // velocity gains

    // Motor power limits - YOUR TUNED VALUES
    public double maxWheelVel = 30.0;        // inches per second
    public double minProfileAccel = -30.0;   // inches per second squared
    public double maxProfileAccel = 30.0;    // inches per second squared

    // Drive constraints - YOUR TUNED VALUES
    public double maxAngVel = Math.PI;       // radians per second
    public double maxAngAccel = Math.PI;     // radians per second squared

    // Track width and wheel base - YOUR MEASURED VALUES
    public double trackWidth = 10.4;         // inches between left and right wheels
    public double wheelBase = 10.125;        // inches between front and back wheels

    // Wheel radius - YOUR MEASURED VALUE
    public double wheelRadius = 2.045;       // inches - your specific wheels

    // Motor ticks per revolution - YOUR MOTOR SPECS
    public double ticksPerRev = 145.1;

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
        fr.setDirection(DcMotorSimple.Direction.FORWARD);
        br.setDirection(DcMotorSimple.Direction.FORWARD);

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

    /**
     * Road Runner style: set drive powers from PoseVelocity2d
     */
    public void setDrivePowers(PoseVelocity2d vel) {
        // FIXED: Remove incorrect Y-flip - OTOS already provides correct coordinates
        // Your OTOS coordinate system: +X = forward, +Y = left, +heading = CCW
        // This matches FTC/Road Runner convention, so no flip needed
        PoseVelocity2d corrected = new PoseVelocity2d(
                new Vector2d(vel.linearVel.x, vel.linearVel.y), // No Y flip needed
                vel.angVel
        );

        // Wrap into Dual
        PoseVelocity2dDual<Time> velDual = PoseVelocity2dDual.constant(corrected, 1);

        // Convert to wheel velocities
        MecanumKinematics.WheelVelocities<Time> wheelVels = kinematics.inverse(velDual);

        // Extract values
        double lf = wheelVels.leftFront.get(0);
        double lb = wheelVels.leftBack.get(0);
        double rf = wheelVels.rightFront.get(0);
        double rb = wheelVels.rightBack.get(0);

        // Normalize
        double max = Math.max(Math.max(Math.abs(lf), Math.abs(lb)),
                Math.max(Math.abs(rf), Math.abs(rb)));
        if (max < 1.0) max = 1.0;

        fl.setPower(lf / max);
        bl.setPower(lb / max);
        fr.setPower(rf / max);
        br.setPower(rb / max);
    }

    /**
     * Direct motor control: raw powers (for TeleOp or test)
     */
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
        final double[] start = {-1};

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

            // if you don't have velocity estimates, just pass zero
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
        final double[] start = {-1};

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

    /**
     * Entry point for Road Runner trajectory building
     */
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
    /**
     * Complete DriveToPositionAction class with all fixes:
     * - Larger tolerances
     * - Gentle PID control
     * - Smart slow-down zones
     * - Stability checking
     * - Timeout protection
     */
    private class DriveToPositionAction implements Action {
        private final Pose2d targetPose;

        // FIXED: Larger tolerances to prevent oscillation and infinite loops
        private final double positionTolerance = 2.5; // inches (was 1.0)
        private final double headingTolerance = Math.toRadians(8); // degrees (was 2°)

        // Timeout and stability tracking
        private long startTime = -1;
        private final long timeoutMs = 10000; // 10 second timeout
        private int stableCount = 0;
        private final int requiredStableCount = 3; // Must be stable for 3 cycles

        public DriveToPositionAction(Pose2d targetPose) {
            this.targetPose = targetPose;
        }

        @Override
        public boolean run(@androidx.annotation.NonNull com.acmerobotics.dashboard.telemetry.TelemetryPacket packet) {
            // Initialize start time on first run
            if (startTime < 0) {
                startTime = System.currentTimeMillis();
            }

            // Update current robot pose
            Pose2d currentPose = updatePoseEstimate();

            // Calculate position and heading errors
            double xError = targetPose.position.x - currentPose.position.x;
            double yError = targetPose.position.y - currentPose.position.y;
            double headingError = targetPose.heading.toDouble() - currentPose.heading.toDouble();

            // Normalize heading error to [-π, π]
            while (headingError > Math.PI) headingError -= 2 * Math.PI;
            while (headingError < -Math.PI) headingError += 2 * Math.PI;

            // Calculate total position error
            double positionError = Math.sqrt(xError * xError + yError * yError);

            // Check for timeout to prevent infinite loops
            long elapsed = System.currentTimeMillis() - startTime;
            if (elapsed > timeoutMs) {
                setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));
                packet.put("Status", "TIMEOUT");
                packet.put("Final_Position_Error", positionError);
                packet.put("Final_Heading_Error", Math.toDegrees(headingError));
                return false; // Action complete due to timeout
            }

            // Check if robot is within tolerance
            boolean withinTolerance = (positionError < positionTolerance &&
                    Math.abs(headingError) < headingTolerance);

            // STABILITY CHECK: Must be stable for multiple cycles to prevent
            // completing during brief oscillations
            if (withinTolerance) {
                stableCount++;
                if (stableCount >= requiredStableCount) {
                    setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));
                    packet.put("Status", "SUCCESS");
                    packet.put("Final_Position_Error", positionError);
                    packet.put("Final_Heading_Error", Math.toDegrees(headingError));
                    packet.put("Completion_Time", elapsed / 1000.0);
                    return false; // Action successfully complete
                }
            } else {
                stableCount = 0; // Reset stability counter if out of tolerance
            }

            // PROPORTIONAL CONTROL: Much gentler gains to prevent oscillation
            double kP_trans = 0.25; // Translational gain (was 0.8)
            double kP_rot = 0.6;    // Rotational gain (was 1.0)

            // Calculate basic drive powers using proportional control
            double x = xError * kP_trans;
            double y = yError * kP_trans;
            double rx = headingError * kP_rot;

            // SPEED LIMITING: Prevent excessive speeds
            double maxSpeed = 0.4; // Maximum speed (was 0.8)
            x = Math.max(-maxSpeed, Math.min(maxSpeed, x));
            y = Math.max(-maxSpeed, Math.min(maxSpeed, y));
            rx = Math.max(-maxSpeed, Math.min(maxSpeed, rx));

            // SMART SLOW-DOWN ZONES: Reduce speed when approaching target

            // Zone 1: Within 2 inches of target - reduce to 50% speed
            if (positionError < 2.0) {
                x *= 0.5;
                y *= 0.5;
            }

            // Zone 2: Within 1 inch of target - reduce to 30% speed
            if (positionError < 1.0) {
                x *= 0.6; // 50% * 60% = 30% of original
                y *= 0.6;
            }

            // Zone 3: Within 0.5 inches of target - reduce to 15% speed
            if (positionError < 0.5) {
                x *= 0.5; // 30% * 50% = 15% of original
                y *= 0.5;
            }

            // Heading slow-down zones
            double headingErrorDegrees = Math.abs(Math.toDegrees(headingError));

            // Within 10 degrees - reduce rotation to 50%
            if (headingErrorDegrees < 10.0) {
                rx *= 0.5;
            }

            // Within 5 degrees - reduce rotation to 25%
            if (headingErrorDegrees < 5.0) {
                rx *= 0.5; // 50% * 50% = 25% of original
            }

            // MINIMUM SPEED: Ensure robot can overcome static friction
            double minSpeed = 0.05;

            // Apply minimum speed if error is significant but calculated speed is too low
            if (Math.abs(xError) > 0.2 && Math.abs(x) > 0 && Math.abs(x) < minSpeed) {
                x = Math.signum(x) * minSpeed;
            }
            if (Math.abs(yError) > 0.2 && Math.abs(y) > 0 && Math.abs(y) < minSpeed) {
                y = Math.signum(y) * minSpeed;
            }
            if (headingErrorDegrees > 2.0 && Math.abs(rx) > 0 && Math.abs(rx) < minSpeed) {
                rx = Math.signum(rx) * minSpeed;
            }

            // Set the calculated drive powers
            setDrivePowers(new PoseVelocity2d(new Vector2d(x, y), rx));

            // COMPREHENSIVE TELEMETRY for debugging and monitoring
            packet.put("Target_X", targetPose.position.x);
            packet.put("Target_Y", targetPose.position.y);
            packet.put("Target_Heading_Deg", Math.toDegrees(targetPose.heading.toDouble()));
            packet.put("Current_X", currentPose.position.x);
            packet.put("Current_Y", currentPose.position.y);
            packet.put("Current_Heading_Deg", Math.toDegrees(currentPose.heading.toDouble()));
            packet.put("Position_Error", positionError);
            packet.put("Heading_Error_Deg", headingErrorDegrees);
            packet.put("Elapsed_Time_Sec", elapsed / 1000.0);
            packet.put("Stable_Count", stableCount);
            packet.put("Within_Tolerance", withinTolerance);
            packet.put("Command_X", x);
            packet.put("Command_Y", y);
            packet.put("Command_RX", rx);
            packet.put("Status", "RUNNING");

            return true; // Continue running the action
        }
    }

    public void updatePIDGains(double axial, double lateral, double heading,
                               double axialVel, double lateralVel, double headingVel) {
        this.controller = new HolonomicController(
                axial, lateral, heading,
                axialVel, lateralVel, headingVel
        );
    }

    public void updateConstraints(double maxVel, double maxAccel,
                                  double maxAngVel, double maxAngAccel) {
        this.maxWheelVel = maxVel;
        this.maxProfileAccel = maxAccel;
        this.maxAngVel = maxAngVel;
        this.maxAngAccel = maxAngAccel;
    }
}