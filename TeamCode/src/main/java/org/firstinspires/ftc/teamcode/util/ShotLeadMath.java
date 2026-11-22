package org.firstinspires.ftc.teamcode.util;

/** Horizontal aiming math for a stationary target and a moving release point. */
public final class ShotLeadMath {
    private ShotLeadMath() {}

    /**
     * Finds the launch velocity relative to the release point, with components along the field axes.
     * The displacement is from ball release position to target. Supply measured or estimated flight
     * time and use a consistent distance unit. This model assumes constant horizontal ball velocity
     * after release. The requested velocity can exceed the shooter's attainable range.
     */
    public static HorizontalVelocity requiredShotVelocity(double displacementX, double displacementY,
            HorizontalVelocity releasePointVelocity, double flightTimeSeconds) {
        if (!Double.isFinite(flightTimeSeconds) || flightTimeSeconds <= 0.0) {
            throw new IllegalArgumentException("Flight time must be finite and positive");
        }
        return new HorizontalVelocity(
                displacementX / flightTimeSeconds - releasePointVelocity.x,
                displacementY / flightTimeSeconds - releasePointVelocity.y);
    }

    /**
     * Adds the linear velocity from a chassis turning in the field plane. The field-relative offset
     * points from the chassis pose origin to the release point, and positive rotation is CCW.
     * If the turret also rotates around an offset pivot, include that additional velocity separately.
     */
    public static HorizontalVelocity chassisVelocityAtRelease(double chassisVelocityX,
            double chassisVelocityY, double chassisAngularVelocityRadiansPerSecond,
            double releaseOffsetX, double releaseOffsetY) {
        return new HorizontalVelocity(
                chassisVelocityX - chassisAngularVelocityRadiansPerSecond * releaseOffsetY,
                chassisVelocityY + chassisAngularVelocityRadiansPerSecond * releaseOffsetX);
    }

    public static final class HorizontalVelocity {
        public final double x;
        public final double y;

        public HorizontalVelocity(double x, double y) {
            if (!Double.isFinite(x) || !Double.isFinite(y)) {
                throw new IllegalArgumentException("Velocity components must be finite");
            }
            this.x = x;
            this.y = y;
        }

        public double speed() {
            return Math.hypot(x, y);
        }

        public double fieldHeadingRadians() {
            if (x == 0.0 && y == 0.0) {
                throw new IllegalStateException("Zero horizontal velocity has no heading");
            }
            return Math.atan2(y, x);
        }

        public double robotRelativeHeadingRadians(double robotHeadingRadians) {
            return MathUtil.wrapAngleRadians(fieldHeadingRadians() - robotHeadingRadians);
        }
    }
}
