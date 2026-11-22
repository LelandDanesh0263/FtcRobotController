package org.firstinspires.ftc.teamcode.util;

/** Rest-to-rest quintic S-curve with time scaled to satisfy speed, acceleration, and jerk limits. */
public final class SCurveMotionProfile implements MotionProfile {
    private final double startPosition;
    private final double endPosition;
    private final double displacement;
    private final double duration;

    public SCurveMotionProfile(double startPosition, double endPosition,
            double maxVelocity, double maxAcceleration, double maxJerk) {
        this.startPosition = startPosition;
        this.endPosition = endPosition;
        displacement = endPosition - startPosition;
        if (!Double.isFinite(displacement) || !positiveFinite(maxVelocity)
                || !positiveFinite(maxAcceleration) || !positiveFinite(maxJerk)) {
            throw new IllegalArgumentException("Motion requires finite positions and positive finite limits");
        }

        // Peaks of the first, second, and third derivatives of 10u^3 - 15u^4 + 6u^5.
        double distance = Math.abs(displacement);
        double velocityTime = (distance / maxVelocity) * (15.0 / 8.0);
        double accelerationTime = Math.sqrt(distance) / Math.sqrt(maxAcceleration)
                * Math.sqrt(10.0 / Math.sqrt(3.0));
        double jerkTime = Math.cbrt(distance) / Math.cbrt(maxJerk) * Math.cbrt(60.0);
        duration = Math.max(velocityTime, Math.max(accelerationTime, jerkTime));
        if (!Double.isFinite(duration) || (distance > 0.0 && duration == 0.0)) {
            throw new IllegalArgumentException("Motion duration is outside the representable range");
        }
    }

    private static boolean positiveFinite(double limit) {
        return Double.isFinite(limit) && limit > 0.0;
    }

    @Override
    public double durationSeconds() {
        return duration;
    }

    @Override
    public State sample(double elapsedSeconds) {
        if (Double.isNaN(elapsedSeconds)) {
            throw new IllegalArgumentException("Elapsed time must be a number");
        }
        if (elapsedSeconds <= 0.0) {
            return new State(startPosition, 0.0, 0.0);
        }
        if (elapsedSeconds >= duration) {
            return new State(endPosition, 0.0, 0.0);
        }

        double u = elapsedSeconds / duration;
        double remaining = 1.0 - u;
        double progress = u * u * u * (10.0 + u * (-15.0 + 6.0 * u));
        double scale = displacement / duration;
        double velocity = scale * (30.0 * u * u * remaining * remaining);
        double acceleration = scale * (60.0 * u * remaining * (1.0 - 2.0 * u)) / duration;
        return new State(startPosition + displacement * progress, velocity, acceleration);
    }
}
