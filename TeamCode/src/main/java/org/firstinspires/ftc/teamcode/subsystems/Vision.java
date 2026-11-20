package org.firstinspires.ftc.teamcode.subsystems;

import android.util.Size;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.RobotLog;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.teamcode.util.BioBuzzTargets;
import org.firstinspires.ftc.teamcode.util.VisionCameraConfig;
import org.firstinspires.ftc.teamcode.util.VisionCameraConfig.ColorProfile;
import org.firstinspires.ftc.teamcode.util.VisionCameraConfig.Detector;
import org.firstinspires.ftc.teamcode.util.VisionData;
import org.firstinspires.ftc.teamcode.util.VisionObservation;
import org.firstinspires.ftc.teamcode.util.VisionObservation.*;
import org.firstinspires.ftc.teamcode.util.VisionSnapshot;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.*;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.firstinspires.ftc.vision.opencv.ImageRegion;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Objects;

/** Optional multi-camera observations shared by TeleOp, autonomous, and mechanisms. */
@Config
public class Vision implements Subsystem {
    // Default BaseRobot opens no camera. Set before INIT, or pass explicit camera configs.
    public static boolean ENABLE_VISION = false;
    public static String WEBCAM_NAME = "Webcam 1";
    public static int STREAM_WIDTH = 640;
    public static int STREAM_HEIGHT = 480;
    public static boolean ENABLE_APRIL_TAGS = true;
    public static boolean ENABLE_COLOR_BLOBS = false;
    public static double MAX_OBSERVATION_AGE_SECONDS = 0.25;

    private final VisionSnapshot snapshot = new VisionSnapshot();
    private final Map<String, Camera> cameras = new LinkedHashMap<>();

    /** Hardware-free empty vision service, also useful for code that does not use vision. */
    public Vision() { this(null, Collections.emptyList()); }

    public Vision(HardwareMap hardwareMap) {
        this(hardwareMap, ENABLE_VISION
                ? Collections.singletonList(defaultConfig(WEBCAM_NAME, ColorRange.BLUE))
                : Collections.emptyList());
    }

    /** An explicitly named camera opts in, independently of the default ENABLE_VISION flag. */
    public Vision(HardwareMap hardwareMap, String webcamName) { this(hardwareMap, webcamName, ColorRange.BLUE); }

    public Vision(HardwareMap hardwareMap, String webcamName, ColorRange blobColor) {
        this(hardwareMap, Collections.singletonList(defaultConfig(webcamName, blobColor)));
    }

    public Vision(HardwareMap hardwareMap, VisionCameraConfig... configs) { this(hardwareMap, Arrays.asList(configs)); }

    public Vision(HardwareMap hardwareMap, List<VisionCameraConfig> configs) {
        Objects.requireNonNull(configs);
        // Validate all identities before opening any hardware.
        Map<String, Boolean> usedWebcams = new LinkedHashMap<>();
        for (VisionCameraConfig config : configs) {
            Objects.requireNonNull(config);
            if (cameras.containsKey(config.id)) throw new IllegalArgumentException("Duplicate camera ID: " + config.id);
            if (config.enabled && usedWebcams.put(config.webcamName, true) != null) {
                throw new IllegalArgumentException("Two active configs use the same webcam: " + config.webcamName);
            }
            cameras.put(config.id, new Camera(config));
        }
        for (Camera camera : cameras.values()) openCamera(hardwareMap, camera);
    }

    private static VisionCameraConfig defaultConfig(String webcamName, ColorRange color) {
        VisionCameraConfig.Builder builder = new VisionCameraConfig.Builder("main", webcamName, CameraRole.GENERAL)
                .resolution(STREAM_WIDTH, STREAM_HEIGHT).aprilTags(ENABLE_APRIL_TAGS);
        if (ENABLE_COLOR_BLOBS) {
            builder.colorCandidate(new ColorProfile("legacy-color", Kind.COLOR_BLOB, Alliance.UNKNOWN, color, 0));
        }
        return builder.build();
    }

    private void openCamera(HardwareMap hardwareMap, Camera camera) {
        if (!camera.config.enabled) { camera.status = "DISABLED"; return; }
        if (hardwareMap == null) { camera.status = "MISSING"; return; }
        try {
            WebcamName webcam = hardwareMap.tryGet(WebcamName.class, camera.config.webcamName);
            if (webcam == null) { camera.status = "MISSING"; return; }
            if (camera.config.aprilTags) {
                camera.tags = new TagDetector(camera.config);
                camera.detectors.put("apriltags", camera.tags);
            }
            for (ColorProfile profile : camera.config.colorProfiles) {
                ColorDetector detector = new ColorDetector(camera.config, profile);
                camera.detectors.put("color/" + profile.id, detector);
                if (camera.firstColor == null) camera.firstColor = detector;
            }
            for (Detector detector : camera.config.extraDetectors) camera.detectors.put("object/" + detector.id, detector);
            VisionPortal.Builder builder = new VisionPortal.Builder().setCamera(webcam)
                    .setCameraResolution(new Size(camera.config.width, camera.config.height))
                    .setLiveViewContainerId(0); // Separate portals, no preview containers or layout assumptions.
            for (Detector detector : camera.detectors.values()) builder.addProcessor(detector);
            camera.portal = builder.build();
            camera.status = "STARTING";
        } catch (RuntimeException e) {
            camera.error = e.getClass().getSimpleName() + ": " + e.getMessage();
            closeCamera(camera, "ERROR");
            RobotLog.ee("Vision", e, "Could not initialize camera '%s'", camera.config.id);
        }
    }

    @Override public void periodic() {
        for (Camera camera : cameras.values()) {
            if (camera.portal == null) continue;
            try {
                camera.status = camera.portal.getCameraState().name();
                if (!isStreaming(camera)) { snapshot.clearCamera(camera.config.id); continue; }
                for (Map.Entry<String, Detector> entry : camera.detectors.entrySet()) {
                    if (isProcessorDisabled(camera, entry.getKey())) {
                        snapshot.clearProcessor(camera.config.id, entry.getKey());
                        continue;
                    }
                    VisionSnapshot.Frame frame = entry.getValue().getLatestFrame();
                    long minimumTime = minimumFrameTime(camera, entry.getKey());
                    if (frame != null && frame.captureTimeNanos >= minimumTime) {
                        snapshot.publish(camera.config.id, entry.getKey(), frame);
                    } else {
                        snapshot.clearProcessor(camera.config.id, entry.getKey());
                    }
                }
            } catch (RuntimeException e) {
                camera.error = e.getClass().getSimpleName() + ": " + e.getMessage();
                closeCamera(camera, "ERROR");
                RobotLog.ee("Vision", e, "Camera update failed: %s", camera.config.id);
            }
        }
    }

    /** Immutable fresh observations; duplicates from different cameras remain separate. */
    public List<VisionObservation> getObservations() {
        return snapshot.getObservations(System.nanoTime(), MAX_OBSERVATION_AGE_SECONDS);
    }

    public List<VisionObservation> getObservations(Kind kind) {
        List<VisionObservation> result = new ArrayList<>();
        for (VisionObservation observation : getObservations()) if (observation.kind == kind) result.add(observation);
        return Collections.unmodifiableList(result);
    }

    public List<VisionObservation> getObservationsFromCamera(String cameraId) {
        List<VisionObservation> result = new ArrayList<>();
        for (VisionObservation observation : getObservations()) {
            if (observation.cameraId.equals(cameraId)) result.add(observation);
        }
        return Collections.unmodifiableList(result);
    }

    /** A detected cell is not proof that it is upward-facing or safe to score into. */
    public List<VisionObservation> getHiveCells(Alliance alliance) {
        List<VisionObservation> result = new ArrayList<>();
        for (VisionObservation observation : getObservations(Kind.HIVE_CELL)) {
            if (observation.alliance == alliance) result.add(observation);
        }
        return Collections.unmodifiableList(result);
    }

    public Map<String, String> getCameraStatuses() {
        Map<String, String> result = new LinkedHashMap<>();
        for (Camera camera : cameras.values()) result.put(camera.config.id, camera.status);
        return Collections.unmodifiableMap(result);
    }

    public Map<String, String> getErrors() {
        Map<String, String> result = new LinkedHashMap<>();
        for (Camera camera : cameras.values()) {
            if (!camera.error.isEmpty()) result.put(camera.config.id, camera.error);
            for (Map.Entry<String, Detector> entry : camera.detectors.entrySet()) {
                if (!entry.getValue().getLastError().isEmpty()) {
                    result.put(camera.config.id + "/" + entry.getKey(), entry.getValue().getLastError());
                }
            }
        }
        return Collections.unmodifiableMap(result);
    }

    public boolean isCameraInitialized() {
        for (Camera camera : cameras.values()) if (isStreaming(camera)) return true;
        return false;
    }

    /** Keys: apriltags, color/<profile ID>, or object/<detector ID>. */
    public void setProcessorEnabled(String cameraId, String processorId, boolean enabled) {
        Camera camera = cameras.get(cameraId);
        if (camera == null) throw new IllegalArgumentException("Unknown camera: " + cameraId);
        if (camera.portal == null) return;
        Detector detector = camera.detectors.get(processorId);
        if (detector == null) throw new IllegalArgumentException("Unknown processor: " + processorId);
        camera.portal.setProcessorEnabled(detector, enabled);
        camera.minimumFrameTimes.put(processorId, System.nanoTime());
        snapshot.clearProcessor(cameraId, processorId);
    }

    // Compatibility helpers: raw SDK getters refer to the first configured camera.
    public VisionPortal getVisionPortal() { Camera c = primary(); return c == null ? null : c.portal; }
    public AprilTagProcessor getAprilTagProcessor() { Camera c = primary(); return c == null || c.tags == null ? null : c.tags.processor; }
    public ColorBlobLocatorProcessor getColorBlobProcessor() { Camera c = primary(); return c == null || c.firstColor == null ? null : c.firstColor.processor; }
    public void setAprilTagProcessorEnabled(boolean enabled) {
        Camera c = primary(); if (c != null && c.tags != null) setProcessorEnabled(c.config.id, "apriltags", enabled);
    }
    public void setColorBlobProcessorEnabled(boolean enabled) {
        Camera c = primary();
        if (c != null) for (String id : c.detectors.keySet()) if (id.startsWith("color/")) setProcessorEnabled(c.config.id, id, enabled);
    }

    public List<AprilTagDetection> getAprilTagDetections() {
        Camera c = primary();
        if (c == null || c.tags == null || isFrameStale(c, "apriltags")) return Collections.emptyList();
        List<AprilTagDetection> result = new ArrayList<>();
        long now = System.nanoTime();
        for (AprilTagDetection d : c.tags.processor.getDetections()) {
            if (VisionObservation.isFresh(d.frameAcquisitionNanoTime, now, MAX_OBSERVATION_AGE_SECONDS)) result.add(d);
        }
        return Collections.unmodifiableList(result);
    }

    public boolean isTagVisible(int tagId) {
        // A partial cluster does not prove that any particular member tag was visible.
        for (VisionObservation o : getObservations(Kind.APRIL_TAG)) if (o.tagIds.contains(tagId)) return true;
        return false;
    }

    /** Single-tag pose only: cluster pose uses a different origin. Use getHiveCells for clusters. */
    public VisionData getAprilTagData(int tagId) {
        for (VisionObservation o : getObservations(Kind.APRIL_TAG)) {
            if (o.tagIds.contains(tagId)) return o.pose.hasPose() ? o.pose : missingPose(tagId, o.label, o.captureTimeNanos);
        }
        return VisionData.NO_TARGET;
    }

    public VisionData getClosestAprilTag() {
        VisionData closest = VisionData.NO_TARGET;
        for (VisionObservation o : getObservations(Kind.APRIL_TAG)) {
            if (o.pose.hasPose() && (!closest.hasPose() || o.pose.rangeInches < closest.rangeInches)) closest = o.pose;
        }
        return closest;
    }

    public List<ColorBlobLocatorProcessor.Blob> getDetectedBlobs() {
        Camera c = primary();
        return c == null || c.firstColor == null || isFrameStale(c, c.firstColor.id)
                ? Collections.emptyList() : Collections.unmodifiableList(new ArrayList<>(c.firstColor.processor.getBlobs()));
    }
    public boolean hasDetectedBlobs() { return !getDetectedBlobs().isEmpty(); }

    private Camera primary() { return cameras.isEmpty() ? null : cameras.values().iterator().next(); }
    private static boolean isStreaming(Camera c) { return c.portal != null && c.portal.getCameraState() == VisionPortal.CameraState.STREAMING; }
    private static boolean isProcessorDisabled(Camera camera, String id) {
        Detector detector = camera.detectors.get(id);
        return detector == null || !isStreaming(camera) || !camera.portal.getProcessorEnabled(detector);
    }

    private static long minimumFrameTime(Camera camera, String id) {
        Long time = camera.minimumFrameTimes.get(id);
        return time == null ? 0L : time;
    }

    private static boolean isFrameStale(Camera camera, String id) {
        Detector detector = camera.detectors.get(id);
        if (detector == null || isProcessorDisabled(camera, id)) return true;
        VisionSnapshot.Frame frame = detector.getLatestFrame();
        return frame == null || frame.captureTimeNanos < minimumFrameTime(camera, id)
                || !VisionObservation.isFresh(frame.captureTimeNanos, System.nanoTime(), MAX_OBSERVATION_AGE_SECONDS);
    }

    @Override public void stop() {
        for (Camera camera : cameras.values()) closeCamera(camera, "STOPPED");
        snapshot.clear();
    }

    private void closeCamera(Camera camera, String status) {
        VisionPortal portal = camera.portal;
        camera.portal = null;
        camera.status = status;
        snapshot.clearCamera(camera.config.id);
        if (portal != null) try { portal.close(); }
        catch (RuntimeException e) { camera.error = "Close failed: " + e.getMessage(); RobotLog.ee("Vision", e, "Could not close camera"); }
        if (!camera.detectorsClosed) {
            camera.detectorsClosed = true;
            for (Detector detector : camera.detectors.values()) {
                try { detector.close(); }
                catch (RuntimeException e) { camera.error = "Detector close failed: " + e.getMessage(); RobotLog.ee("Vision", e, "Could not close detector"); }
            }
        }
    }

    private static final class Camera {
        final VisionCameraConfig config;
        final Map<String, Detector> detectors = new LinkedHashMap<>();
        final Map<String, Long> minimumFrameTimes = new LinkedHashMap<>();
        VisionPortal portal;
        TagDetector tags;
        ColorDetector firstColor;
        boolean detectorsClosed;
        String status = "UNINITIALIZED", error = "";
        Camera(VisionCameraConfig config) { this.config = config; }
    }

    private static final class TagDetector extends Detector {
        final AprilTagProcessor processor;
        TagDetector(VisionCameraConfig config) {
            super("apriltags", config.id, config.role);
            processor = new AprilTagProcessor.Builder().setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES).build();
        }
        @Override protected void initialize(int width, int height, CameraCalibration calibration) { processor.init(width, height, calibration); }
        @Override protected List<VisionObservation> detect(Mat frame, long timestamp) {
            processor.processFrame(frame, timestamp);
            return describeAprilTags(cameraId, cameraRole, timestamp, processor.getDetections());
        }
    }

    /** Converts SDK results without conflating a cluster origin and an individual tag origin. */
    static List<VisionObservation> describeAprilTags(String cameraId, CameraRole role, long timestamp, List<AprilTagDetection> detections) {
        List<VisionObservation> result = new ArrayList<>();
        for (AprilTagDetection detection : detections) {
            VisionObservation.Builder builder;
            BioBuzzTargets.Cell cell;
            if (detection instanceof AprilTagSingleDetection) {
                AprilTagSingleDetection tag = (AprilTagSingleDetection) detection;
                String name = tag.metadata == null ? "Unknown tag" : tag.metadata.name;
                builder = new VisionObservation.Builder(cameraId, role, Kind.APRIL_TAG, Evidence.APRIL_TAG, timestamp)
                        .label(name).tagIds(tag.id).targetId("tag/" + tag.id)
                        .pose(tagPose(tag, tag.id, name, timestamp), PoseOrigin.SINGLE_TAG);
                cell = BioBuzzTargets.forTagId(tag.id);
                if (cell != null) builder.alliance(cell.alliance);
                if (tag.corners != null && tag.corners.length > 0) {
                    double left = Double.POSITIVE_INFINITY;
                    double top = Double.POSITIVE_INFINITY;
                    double right = Double.NEGATIVE_INFINITY;
                    double bottom = Double.NEGATIVE_INFINITY;
                    for (Point p : tag.corners) { left = Math.min(left, p.x); top = Math.min(top, p.y); right = Math.max(right, p.x); bottom = Math.max(bottom, p.y); }
                    builder.bounds(new ImageBounds(left, top, right, bottom));
                }
            } else if (detection instanceof AprilTagClusterDetection) {
                AprilTagClusterDetection cluster = (AprilTagClusterDetection) detection;
                String name = cluster.metadata == null ? "Unknown cluster" : cluster.metadata.name;
                cell = BioBuzzTargets.forSdkClusterName(name);
                builder = new VisionObservation.Builder(cameraId, role, cell == null ? Kind.APRIL_TAG_CLUSTER : Kind.HIVE_CELL,
                        Evidence.TAG_CLUSTER, timestamp).label(name).clusterCoveragePercent(cluster.percentClusterFound)
                        .pose(tagPose(cluster, -1, name, timestamp), PoseOrigin.TAG_CLUSTER);
                if (cell != null) builder.targetId(cell.targetId).alliance(cell.alliance).tagIds(cell.tagIds());
            } else continue;
            result.add(builder.build());
        }
        return result;
    }

    private static VisionData tagPose(AprilTagDetection detection, int id, String name, long timestamp) {
        AprilTagPoseFtc p = detection.ftcPose;
        return p == null ? missingPose(id, name, timestamp) : new VisionData(true, id, name, timestamp,
                p.range, p.bearing, p.elevation, p.yaw, p.pitch, p.roll, p.x, p.y, p.z);
    }
    private static VisionData missingPose(int id, String name, long timestamp) {
        return new VisionData(true, id, name, timestamp, Double.NaN, Double.NaN, Double.NaN,
                Double.NaN, Double.NaN, Double.NaN, Double.NaN, Double.NaN, Double.NaN);
    }

    private static final class ColorDetector extends Detector {
        final ColorProfile profile;
        final ColorBlobLocatorProcessor processor;
        ColorDetector(VisionCameraConfig config, ColorProfile profile) {
            super("color/" + profile.id, config.id, config.role);
            this.profile = profile;
            processor = new ColorBlobLocatorProcessor.Builder().setTargetColorRange(profile.colorRange)
                    .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
                    .setRoi(ImageRegion.entireFrame()).setDrawContours(false).build();
        }
        @Override protected void initialize(int width, int height, CameraCalibration calibration) { processor.init(width, height, calibration); }
        @Override protected List<VisionObservation> detect(Mat frame, long timestamp) {
            processor.processFrame(frame, timestamp);
            List<VisionObservation> result = new ArrayList<>();
            for (ColorBlobLocatorProcessor.Blob blob : processor.getBlobs()) {
                if (blob.getContourArea() < profile.minAreaPixels) continue;
                Rect box = blob.getBoxFit().boundingRect();
                result.add(new VisionObservation.Builder(cameraId, cameraRole, profile.candidateKind, Evidence.COLOR_CANDIDATE, timestamp)
                        .label(profile.id).alliance(profile.alliance)
                        .bounds(new ImageBounds(box.x, box.y, box.x + box.width, box.y + box.height)).build());
            }
            return result;
        }
    }
}
