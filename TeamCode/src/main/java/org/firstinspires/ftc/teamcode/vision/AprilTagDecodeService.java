package org.firstinspires.ftc.teamcode.vision;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

// IMPORTANT: use the FTC SDK VisionPortal AprilTag classes (NOT org.openftc.*)
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.Optional;

public class AprilTagDecodeService {
    public enum Slot { LEFT, CENTER, RIGHT, NONE }

    private final HardwareMap hw;
    private final String webcamConfigName;

    private VisionPortal portal;
    private AprilTagProcessor processor;
    private AprilTagLibrary library;

    // Configure these for the Decode season
    private int idLeft = 1, idCenter = 2, idRight = 3;
    private double tagSizeMeters = 0.165; // ~6.5 in

    // Sticky memory to ride out brief dropouts
    private long holdMillis = 300;
    private Slot lastSlot = Slot.NONE;
    private int lastId = -1;
    private long lastUpdateMs = 0L;

    public AprilTagDecodeService(HardwareMap hw, String webcamConfigName) {
        this.hw = hw;
        this.webcamConfigName = (webcamConfigName != null ? webcamConfigName : "Webcam 1");
    }

    // ---------- Configuration ----------
    public AprilTagDecodeService setExpectedIds(int left, int center, int right) {
        this.idLeft = left; this.idCenter = center; this.idRight = right; return this;
    }
    public AprilTagDecodeService setTagSizeMeters(double meters) {
        this.tagSizeMeters = meters; return this;
    }
    public AprilTagDecodeService setHoldMillis(long ms) {
        this.holdMillis = Math.max(0, ms); return this;
    }

    /** Prefer the SDK's current season library; if unavailable, build a custom 3-tag library. */
    public AprilTagDecodeService useSeasonLibraryOrFallback() {
        try {
            library = AprilTagGameDatabase.getCurrentGameTagLibrary();
        } catch (Throwable t) {
            library = null;
        }
        if (library == null) {
            library = new AprilTagLibrary.Builder()
                    .addTag(idLeft, "left" , tagSizeMeters, DistanceUnit.METER)
                    .addTag(idCenter,"center", tagSizeMeters, DistanceUnit.METER)
                    .addTag(idRight, "right",  tagSizeMeters, DistanceUnit.METER)
                    .build();
        }
        return this;
    }

    // ---------- Lifecycle ----------
    public void init() {
        if (library == null) useSeasonLibraryOrFallback();
        processor = new AprilTagProcessor.Builder()
                .setTagLibrary(library)
                .setDrawAxes(true)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setDrawCubeProjection(true)
                .build();
        portal = new VisionPortal.Builder()
                .setCamera(hw.get(WebcamName.class, webcamConfigName))
                .addProcessor(processor)
                .build();
    }
    public void pause()  { if (portal != null) portal.stopStreaming(); }
    public void resume() { if (portal != null) portal.resumeStreaming(); }
    public void close()  { if (portal != null) portal.close(); }

    // ---------- Queries ----------
    /** Use FTC SDK processor API (NOT OpenFTC JNI). */
    public List<AprilTagDetection> getDetections() {
        return (processor != null) ? processor.getDetections() : List.of();
    }

    /** Fast-path for “only one tag visible at a time”. */
    public Slot getSeenSlot() {
        List<AprilTagDetection> ds = getDetections();
        for (AprilTagDetection d : ds) {
            Slot s = slotForId(d.id);
            if (s != Slot.NONE) {
                lastSlot = s; lastId = d.id; lastUpdateMs = System.currentTimeMillis();
                return s;
            }
        }
        if (holdMillis > 0 && (System.currentTimeMillis() - lastUpdateMs) <= holdMillis) {
            return lastSlot;
        }
        return Slot.NONE;
    }

    public int getSeenId() {
        Slot s = getSeenSlot();
        switch (s) {
            case LEFT:   return idLeft;
            case CENTER: return idCenter;
            case RIGHT:  return idRight;
            default:     return -1;
        }
    }

    /** Wait until one of the three expected tags is seen or timeout (ms) expires. */
    public Slot waitForSlot(long timeoutMs) {
        long end = System.currentTimeMillis() + Math.max(0, timeoutMs);
        Slot s;
        while ((s = getSeenSlot()) == Slot.NONE && System.currentTimeMillis() < end) {
            try { Thread.sleep(10); } catch (InterruptedException ignored) {}
        }
        return s;
    }

    public Optional<AprilTagDetection> getClosestExpected() {
        // With only-one-visible assumption, this is effectively the first expected detection.
        for (AprilTagDetection d : getDetections()) {
            if (slotForId(d.id) != Slot.NONE) return Optional.of(d);
        }
        return Optional.empty();
    }

    public void addTelemetry(Telemetry tl) {
        List<AprilTagDetection> dets = getDetections();
        tl.addData("Detections", dets.size());
        for (AprilTagDetection d : dets) {
            tl.addData("ID", d.id);
            if (d.ftcPose != null) {
                tl.addData("   Range (in)", "%.1f", d.ftcPose.range);
                tl.addData("   Bearing (deg)", "%.1f", d.ftcPose.bearing);
            }
        }
        tl.addData("Chosen Slot (sticky)", getSeenSlot());
        tl.addData("Chosen ID", getSeenId());
    }

    // ---------- Helpers ----------
    private Slot slotForId(int id) {
        if (id == idLeft)   return Slot.LEFT;
        if (id == idCenter) return Slot.CENTER;
        if (id == idRight)  return Slot.RIGHT;
        return Slot.NONE;
    }
}
