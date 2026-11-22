package org.firstinspires.ftc.teamcode.util;

/**
 * Ideal release math for a ball in simultaneous, no-slip contact with opposite wheels.
 * Surface speeds are signed along the shot direction at the ball contact, so both treads
 * moving toward the exit have positive speeds even though their axles spin in opposite senses.
 * Use the same length unit for all radii and distances per second. Positive ball angular
 * velocity is backspin, meaning the bottom of the ball moves toward the exit relative to
 * its center. Actual release speed and spin depend on grip and compression.
 */
public final class BackspinMath {
    private static final double RADIANS_PER_SECOND_PER_RPM = 2.0 * Math.PI / 60.0;

    private BackspinMath() {}

    /** Converts signed wheel RPM to tread speed using the wheel's effective contact radius. */
    public static double wheelSurfaceSpeed(double wheelRpm, double wheelRadius) {
        requirePositiveRadius(wheelRadius);
        return wheelRpm * RADIANS_PER_SECOND_PER_RPM * wheelRadius;
    }

    /** Converts tread speed to signed wheel RPM in the same contact-direction convention. */
    public static double wheelRpm(double surfaceSpeed, double wheelRadius) {
        requirePositiveRadius(wheelRadius);
        return surfaceSpeed / wheelRadius / RADIANS_PER_SECOND_PER_RPM;
    }

    /** Returns the ball center's ideal forward speed in length units per second. */
    public static double exitSpeed(double upperSurfaceSpeed, double lowerSurfaceSpeed) {
        return 0.5 * upperSurfaceSpeed + 0.5 * lowerSurfaceSpeed;
    }

    /** Returns signed ball spin in radians per second. Negative values represent topspin. */
    public static double backspinRadiansPerSecond(
            double upperSurfaceSpeed, double lowerSurfaceSpeed, double ballRadius) {
        requirePositiveRadius(ballRadius);
        return (0.5 * lowerSurfaceSpeed - 0.5 * upperSurfaceSpeed) / ballRadius;
    }

    /** Returns the upper tread speed for a requested exit speed and signed spin. */
    public static double upperSurfaceSpeed(
            double exitSpeed, double backspinRadiansPerSecond, double ballRadius) {
        requirePositiveRadius(ballRadius);
        return exitSpeed - backspinRadiansPerSecond * ballRadius;
    }

    /** Returns the lower tread speed for a requested exit speed and signed spin. */
    public static double lowerSurfaceSpeed(
            double exitSpeed, double backspinRadiansPerSecond, double ballRadius) {
        requirePositiveRadius(ballRadius);
        return exitSpeed + backspinRadiansPerSecond * ballRadius;
    }

    private static void requirePositiveRadius(double radius) {
        if (!Double.isFinite(radius) || radius <= 0.0) {
            throw new IllegalArgumentException("Radius must be finite and positive");
        }
    }
}
