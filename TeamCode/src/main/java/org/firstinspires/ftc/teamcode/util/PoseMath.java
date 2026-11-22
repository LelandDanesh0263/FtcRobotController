package org.firstinspires.ftc.teamcode.util;

import com.pedropathing.geometry.Pose;

/** Rigid transforms in Pedro's planar convention, using one distance unit and CCW radians. */
public final class PoseMath {
    private PoseMath() {}

    /** Maps a local pose through its parent pose. Useful for a calibrated camera or turret mount. */
    public static Pose compose(Pose parent, Pose local) {
        double cos = Math.cos(parent.getHeading());
        double sin = Math.sin(parent.getHeading());
        return new Pose(
                parent.getX() + cos * local.getX() - sin * local.getY(),
                parent.getY() + sin * local.getX() + cos * local.getY(),
                MathUtil.wrapAngleRadians(parent.getHeading() + local.getHeading()));
    }

    /** Reverses a transform, e.g. from robot-to-camera to camera-to-robot. */
    public static Pose inverse(Pose transform) {
        double cos = Math.cos(transform.getHeading());
        double sin = Math.sin(transform.getHeading());
        return new Pose(
                -cos * transform.getX() - sin * transform.getY(),
                sin * transform.getX() - cos * transform.getY(),
                MathUtil.wrapAngleRadians(-transform.getHeading()));
    }

    /**
     * Blends two accepted field poses at the same timestamp. The caller handles stale data,
     * rejects outliers and uses stationary landmarks for absolute field measurements.
     */
    public static Pose blend(Pose estimate, Pose measurement, double measurementWeight) {
        if (!Double.isFinite(measurementWeight) || measurementWeight < 0.0 || measurementWeight > 1.0) {
            throw new IllegalArgumentException("Measurement weight must be between zero and one");
        }
        double headingError = MathUtil.wrapAngleRadians(measurement.getHeading() - estimate.getHeading());
        return new Pose(
                MathUtil.lerp(estimate.getX(), measurement.getX(), measurementWeight),
                MathUtil.lerp(estimate.getY(), measurement.getY(), measurementWeight),
                MathUtil.wrapAngleRadians(estimate.getHeading() + measurementWeight * headingError));
    }
}
