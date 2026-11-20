package org.firstinspires.ftc.teamcode.util;

import java.util.Locale;

/** An immutable observation of one AprilTag. Pose values are relative to the camera. */
public final class VisionData {
    public static final VisionData NO_TARGET = new VisionData(
            false, -1, "Unknown", 0, Double.NaN, Double.NaN, Double.NaN,
            Double.NaN, Double.NaN, Double.NaN, Double.NaN, Double.NaN, Double.NaN);

    public final boolean targetFound;
    public final int id;
    public final String name;
    public final long frameAcquisitionNanoTime;
    public final double rangeInches;
    public final double bearingDegrees;
    public final double elevationDegrees;
    public final double yawDegrees;
    public final double pitchDegrees;
    public final double rollDegrees;
    public final double xInches;
    public final double yInches;
    public final double zInches;

    public VisionData(boolean targetFound, int id, String name, long frameAcquisitionNanoTime,
                      double rangeInches, double bearingDegrees, double elevationDegrees,
                      double yawDegrees, double pitchDegrees, double rollDegrees,
                      double xInches, double yInches, double zInches) {
        this.targetFound = targetFound;
        this.id = id;
        this.name = name;
        this.frameAcquisitionNanoTime = frameAcquisitionNanoTime;
        this.rangeInches = rangeInches;
        this.bearingDegrees = bearingDegrees;
        this.elevationDegrees = elevationDegrees;
        this.yawDegrees = yawDegrees;
        this.pitchDegrees = pitchDegrees;
        this.rollDegrees = rollDegrees;
        this.xInches = xInches;
        this.yInches = yInches;
        this.zInches = zInches;
    }

    /** Unknown tag sizes or unavailable pose estimates leave pose values as NaN. */
    public boolean hasPose() {
        return targetFound && Double.isFinite(rangeInches) && rangeInches >= 0;
    }

    @Override
    public String toString() {
        if (!targetFound) {
            return "No AprilTag detected";
        }
        if (!hasPose()) {
            return String.format(Locale.US, "Tag #%d (%s), pose unavailable", id, name);
        }
        return String.format(Locale.US, "Tag #%d (%s), range %.1f in, bearing %.1f°",
                id, name, rangeInches, bearingDegrees);
    }
}
