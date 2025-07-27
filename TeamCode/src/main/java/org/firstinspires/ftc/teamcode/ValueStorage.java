package org.firstinspires.ftc.teamcode;
import com.acmerobotics.roadrunner.Pose2d;
public class ValueStorage {
    public static final double armRest = 0.20;
    public static final double armDown = 0.515;
    public static final double armUp = 0.29;
    public static final double clawOpen = 0.38;
    public static final double clawClosed = 0.71;
    public static final double bucketRest = 0.28;
    public static final double bucketUp = 0.58;
    public static final double bucketDown = 0.80;
    public static final double gateDown = 0.56;
    public static final double gateUp = 0.80;
    public static final double bucketSensorThreshold = 4.5;
    public static final double[] armStateTimes = {1000, 1000, 1000, 1000, 1000, 1000};
    public static final double[][] intakeStateTimes = {{500, 750, 1000, 0, 750}, {250, 250, 250, 0, 250}, {250, 250, 500, 0, 250}, {250, 250, 250, 0, 250}};
    public static final int bucketDetectionFrames = 10;
    public static final int cameraDetectionFrames = 10;
    public static final int[] liftPositions = {0, -400, -940, -150, -200};//0 = low level, 1 = middle level, 2 = high level, 3 = angled bucket position 4 = between low and mid
    public static final boolean parkConfig = true; //false = stop at warehouse entrance, true = go to shared hub side
    public static int lastIntakeState = 0;
    public static int lastLiftState = 0;
    public static int lastArmState = 0;
    public static int redMultiplier = 1;
    public static Pose2d lastPose = new Pose2d(0, 0, 0);
}
