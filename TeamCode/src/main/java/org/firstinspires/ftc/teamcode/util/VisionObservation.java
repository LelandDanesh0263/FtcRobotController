package org.firstinspires.ftc.teamcode.util;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Objects;

/** One camera observation, not a fused field position or a persistent object track. */
public final class VisionObservation {
    public enum Kind { APRIL_TAG, APRIL_TAG_CLUSTER, POLLEN, NECTAR, FLOWER, HIVE, HIVE_CELL, ROBOT, COLOR_BLOB, UNKNOWN }
    public enum Alliance { RED, BLUE, NEUTRAL, UNKNOWN }
    public enum CameraRole { GENERAL, FIELD, TURRET }
    public enum Evidence { APRIL_TAG, TAG_CLUSTER, COLOR_CANDIDATE, OBJECT_DETECTOR }
    public enum PoseOrigin { NONE, SINGLE_TAG, TAG_CLUSTER, OBJECT }

    public final String cameraId;
    public final CameraRole cameraRole;
    public final Kind kind;
    public final Alliance alliance;
    public final Evidence evidence;
    public final String label;
    /** Optional semantic identity, e.g. red/audience; empty for unidentified objects. */
    public final String targetId;
    /** Cluster membership does not mean every member was visible in this frame. */
    public final List<Integer> tagIds;
    public final long captureTimeNanos;
    /** NaN means the detector did not provide a calibrated confidence score. */
    public final double confidence;
    public final int clusterCoveragePercent;
    public final ImageBounds imageBounds;
    /** Camera-relative pose of poseOrigin, never an inferred field/robot coordinate. */
    public final VisionData pose;
    public final PoseOrigin poseOrigin;

    private VisionObservation(Builder b) {
        cameraId = b.cameraId;
        cameraRole = b.cameraRole;
        kind = b.kind;
        alliance = b.alliance;
        evidence = b.evidence;
        label = b.label;
        targetId = b.targetId;
        tagIds = Collections.unmodifiableList(new ArrayList<>(b.tagIds));
        captureTimeNanos = b.captureTimeNanos;
        confidence = b.confidence;
        clusterCoveragePercent = b.clusterCoveragePercent;
        imageBounds = b.imageBounds;
        pose = b.pose;
        poseOrigin = b.poseOrigin;
    }

    public boolean isFresh(long nowNanos, double maxAgeSeconds) {
        return isFresh(captureTimeNanos, nowNanos, maxAgeSeconds);
    }

    public static boolean isFresh(long captureTimeNanos, long nowNanos, double maxAgeSeconds) {
        if (!Double.isFinite(maxAgeSeconds) || maxAgeSeconds < 0) {
            throw new IllegalArgumentException("Maximum age must be finite and nonnegative");
        }
        long age = nowNanos - captureTimeNanos;
        return captureTimeNanos > 0 && age >= 0 && age / 1e9 <= maxAgeSeconds;
    }

    /** Pixels in this camera's image; a box alone does not establish distance. */
    public static final class ImageBounds {
        public final double left, top, right, bottom;

        public ImageBounds(double left, double top, double right, double bottom) {
            if (!Double.isFinite(left) || !Double.isFinite(top) || !Double.isFinite(right)
                    || !Double.isFinite(bottom) || right < left || bottom < top) {
                throw new IllegalArgumentException("Invalid image bounds");
            }
            this.left = left;
            this.top = top;
            this.right = right;
            this.bottom = bottom;
        }

        public double centerX() { return (left + right) / 2; }
        public double centerY() { return (top + bottom) / 2; }
    }

    public static final class Builder {
        private final String cameraId;
        private final CameraRole cameraRole;
        private final Kind kind;
        private final Evidence evidence;
        private final long captureTimeNanos;
        private Alliance alliance = Alliance.UNKNOWN;
        private String label = "";
        private String targetId = "";
        private final List<Integer> tagIds = new ArrayList<>();
        private double confidence = Double.NaN;
        private int clusterCoveragePercent = -1;
        private ImageBounds imageBounds;
        private VisionData pose = VisionData.NO_TARGET;
        private PoseOrigin poseOrigin = PoseOrigin.NONE;

        public Builder(String cameraId, CameraRole cameraRole, Kind kind, Evidence evidence, long captureTimeNanos) {
            if (cameraId == null || cameraId.trim().isEmpty() || captureTimeNanos <= 0) {
                throw new IllegalArgumentException("Camera identity and capture timestamp are required");
            }
            this.cameraId = cameraId;
            this.cameraRole = Objects.requireNonNull(cameraRole);
            this.kind = Objects.requireNonNull(kind);
            this.evidence = Objects.requireNonNull(evidence);
            this.captureTimeNanos = captureTimeNanos;
        }

        public Builder alliance(Alliance value) { alliance = Objects.requireNonNull(value); return this; }
        public Builder label(String value) { label = Objects.requireNonNull(value); return this; }
        public Builder targetId(String value) { targetId = Objects.requireNonNull(value); return this; }
        public Builder bounds(ImageBounds value) { imageBounds = Objects.requireNonNull(value); return this; }

        public Builder tagIds(int... ids) {
            for (int id : ids) {
                if (id < 0) throw new IllegalArgumentException("Tag IDs must be nonnegative");
                if (!tagIds.contains(id)) tagIds.add(id);
            }
            return this;
        }

        public Builder confidence(double value) {
            if (!Double.isNaN(value) && (!Double.isFinite(value) || value < 0 || value > 1)) {
                throw new IllegalArgumentException("Confidence must be 0..1 or NaN");
            }
            confidence = value;
            return this;
        }

        public Builder clusterCoveragePercent(int value) {
            if (value < 0 || value > 100) throw new IllegalArgumentException("Coverage must be 0..100");
            clusterCoveragePercent = value;
            return this;
        }

        public Builder pose(VisionData value, PoseOrigin origin) {
            Objects.requireNonNull(value);
            Objects.requireNonNull(origin);
            if (value.hasPose()) {
                if (origin == PoseOrigin.NONE || value.frameAcquisitionNanoTime != captureTimeNanos) {
                    throw new IllegalArgumentException("Pose origin and timestamp must match the observation");
                }
                pose = value;
                poseOrigin = origin;
            }
            return this;
        }

        public VisionObservation build() { return new VisionObservation(this); }
    }
}
