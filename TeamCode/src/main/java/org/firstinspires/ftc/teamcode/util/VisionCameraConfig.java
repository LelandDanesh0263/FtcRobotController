package org.firstinspires.ftc.teamcode.util;

import android.graphics.Canvas;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.opencv.core.Mat;
import org.firstinspires.ftc.teamcode.util.VisionObservation.Alliance;
import org.firstinspires.ftc.teamcode.util.VisionObservation.CameraRole;
import org.firstinspires.ftc.teamcode.util.VisionObservation.Kind;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Objects;

/** Explicit optional camera configuration; adding a second camera does not change consumers. */
public final class VisionCameraConfig {
    public final String id, webcamName;
    public final CameraRole role;
    public final int width, height;
    public final boolean enabled, aprilTags;
    public final List<ColorProfile> colorProfiles;
    public final List<Detector> extraDetectors;

    private VisionCameraConfig(Builder b) {
        id = b.id; webcamName = b.webcamName; role = b.role;
        width = b.width; height = b.height; enabled = b.enabled; aprilTags = b.aprilTags;
        colorProfiles = Collections.unmodifiableList(new ArrayList<>(b.colorProfiles));
        extraDetectors = Collections.unmodifiableList(new ArrayList<>(b.extraDetectors));
    }

    public static final class Builder {
        private final String id, webcamName;
        private final CameraRole role;
        private int width = 640, height = 480;
        private boolean enabled = true, aprilTags = true;
        private final List<ColorProfile> colorProfiles = new ArrayList<>();
        private final List<Detector> extraDetectors = new ArrayList<>();

        public Builder(String id, String webcamName, CameraRole role) {
            this.id = requireId(id); this.webcamName = requireId(webcamName);
            this.role = Objects.requireNonNull(role);
        }
        public Builder resolution(int width, int height) {
            if (width <= 0 || height <= 0) throw new IllegalArgumentException("Invalid resolution");
            this.width = width; this.height = height; return this;
        }
        public Builder enabled(boolean enabled) { this.enabled = enabled; return this; }
        public Builder aprilTags(boolean enabled) { aprilTags = enabled; return this; }
        public Builder colorCandidate(ColorProfile profile) {
            Objects.requireNonNull(profile);
            for (ColorProfile existing : colorProfiles) {
                if (existing.id.equals(profile.id)) throw new IllegalArgumentException("Duplicate color profile");
            }
            colorProfiles.add(profile); return this;
        }
        public Builder detector(Detector detector) {
            Objects.requireNonNull(detector);
            if (!id.equals(detector.cameraId) || role != detector.cameraRole) {
                throw new IllegalArgumentException("Detector must use this camera's identity and role");
            }
            for (Detector existing : extraDetectors) {
                if (existing.id.equals(detector.id)) throw new IllegalArgumentException("Duplicate detector");
            }
            extraDetectors.add(detector); return this;
        }
        public VisionCameraConfig build() { return new VisionCameraConfig(this); }
    }

    /** Color-only candidates, not validated recognition of a ball, flower, or robot. */
    public static final class ColorProfile {
        public final String id;
        public final Kind candidateKind;
        public final Alliance alliance;
        public final ColorRange colorRange;
        public final double minAreaPixels;

        public ColorProfile(String id, Kind candidateKind, Alliance alliance, ColorRange colorRange, double minAreaPixels) {
            this.id = requireId(id);
            this.candidateKind = Objects.requireNonNull(candidateKind);
            this.alliance = Objects.requireNonNull(alliance);
            this.colorRange = Objects.requireNonNull(colorRange);
            if (!Double.isFinite(minAreaPixels) || minAreaPixels < 0) throw new IllegalArgumentException("Invalid area");
            this.minAreaPixels = minAreaPixels;
        }
    }

    /** Extend this for a trained model or custom OpenCV detector, once one is selected. */
    public abstract static class Detector implements VisionProcessor {
        public final String id, cameraId;
        public final CameraRole cameraRole;
        protected int imageWidth, imageHeight;
        private volatile VisionSnapshot.Frame latestFrame;
        private volatile String lastError = "";
        private boolean closed;

        protected Detector(String id, String cameraId, CameraRole role) {
            this.id = requireId(id); this.cameraId = requireId(cameraId);
            cameraRole = Objects.requireNonNull(role);
        }
        @Override public final synchronized void init(int width, int height, CameraCalibration calibration) {
            if (closed) return;
            imageWidth = width; imageHeight = height;
            initialize(width, height, calibration);
        }
        protected void initialize(int width, int height, CameraCalibration calibration) {}

        @Override public final synchronized Object processFrame(Mat frame, long captureTimeNanos) {
            if (closed) return null;
            if (captureTimeNanos <= 0) {
                latestFrame = null;
                lastError = "Invalid frame capture timestamp";
                return null;
            }
            try {
                List<VisionObservation> detections = detect(frame, captureTimeNanos);
                for (VisionObservation detection : detections) {
                    if (!cameraId.equals(detection.cameraId) || cameraRole != detection.cameraRole) {
                        throw new IllegalArgumentException("Detector returned another camera's observation");
                    }
                }
                latestFrame = new VisionSnapshot.Frame(captureTimeNanos, detections);
                lastError = "";
            } catch (RuntimeException e) {
                // A failed frame replaces earlier results instead of leaving a stale target active.
                latestFrame = new VisionSnapshot.Frame(captureTimeNanos, Collections.emptyList());
                lastError = e.getClass().getSimpleName() + ": " + String.valueOf(e.getMessage());
            }
            return null;
        }
        protected abstract List<VisionObservation> detect(Mat frame, long captureTimeNanos);
        public final VisionSnapshot.Frame getLatestFrame() { return latestFrame; }
        public final String getLastError() { return lastError; }
        /** Serializes cleanup with frame processing, since VisionPortal.close() is asynchronous. */
        public final synchronized void close() {
            if (closed) return;
            closed = true;
            latestFrame = null;
            release();
        }
        /** Override to release a model/native resources; no later frame will call detect(). */
        protected void release() {}
        @Override public void onDrawFrame(Canvas canvas, int width, int height, float scale, float density, Object context) {}
    }

    private static String requireId(String id) {
        if (id == null || id.trim().isEmpty()) throw new IllegalArgumentException("An identity is required");
        return id;
    }
}
