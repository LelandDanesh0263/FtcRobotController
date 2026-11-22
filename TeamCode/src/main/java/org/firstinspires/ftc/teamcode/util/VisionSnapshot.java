package org.firstinspires.ftc.teamcode.util;

import java.util.ArrayList;
import java.util.Collections;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Objects;

/** Latest complete frame per camera/processor; no cross-camera fusion or object counting. */
public final class VisionSnapshot {
    private final Map<String, Map<String, Frame>> frames = new LinkedHashMap<>();

    public static final class Frame {
        public final long captureTimeNanos;
        public final List<VisionObservation> observations;

        public Frame(long captureTimeNanos, List<VisionObservation> observations) {
            if (captureTimeNanos <= 0) throw new IllegalArgumentException("Capture timestamp is required");
            List<VisionObservation> copy = new ArrayList<>(Objects.requireNonNull(observations));
            for (VisionObservation observation : copy) {
                if (observation == null || observation.captureTimeNanos != captureTimeNanos) {
                    throw new IllegalArgumentException("Every observation must belong to this frame");
                }
            }
            this.captureTimeNanos = captureTimeNanos;
            this.observations = Collections.unmodifiableList(copy);
        }
    }

    /** Publish the whole frame, including an empty list when nothing was detected. */
    public synchronized boolean publish(String cameraId, String processorId, Frame frame) {
        requireId(cameraId);
        requireId(processorId);
        Objects.requireNonNull(frame);
        for (VisionObservation observation : frame.observations) {
            if (!cameraId.equals(observation.cameraId)) {
                throw new IllegalArgumentException("Observation belongs to a different camera");
            }
        }
        Map<String, Frame> cameraFrames = frames.get(cameraId);
        if (cameraFrames == null) {
            cameraFrames = new LinkedHashMap<>();
            frames.put(cameraId, cameraFrames);
        }
        Frame previous = cameraFrames.get(processorId);
        if (previous != null && frame.captureTimeNanos <= previous.captureTimeNanos) return false;
        cameraFrames.put(processorId, frame);
        return true;
    }

    /** Time is supplied explicitly so freshness can be tested without a camera or robot. */
    public synchronized List<VisionObservation> getObservations(long nowNanos, double maxAgeSeconds) {
        // Validate the setting even when no cameras have produced a frame.
        VisionObservation.isFresh(1, nowNanos, maxAgeSeconds);
        List<VisionObservation> result = new ArrayList<>();
        for (Map<String, Frame> cameraFrames : frames.values()) {
            for (Frame frame : cameraFrames.values()) {
                if (VisionObservation.isFresh(frame.captureTimeNanos, nowNanos, maxAgeSeconds)) {
                    result.addAll(frame.observations);
                }
            }
        }
        return Collections.unmodifiableList(result);
    }

    public synchronized void clearProcessor(String cameraId, String processorId) {
        Map<String, Frame> cameraFrames = frames.get(cameraId);
        if (cameraFrames != null) cameraFrames.remove(processorId);
    }

    public synchronized void clearCamera(String cameraId) { frames.remove(cameraId); }
    public synchronized void clear() { frames.clear(); }

    private static void requireId(String id) {
        if (id == null || id.trim().isEmpty()) throw new IllegalArgumentException("An identity is required");
    }
}
