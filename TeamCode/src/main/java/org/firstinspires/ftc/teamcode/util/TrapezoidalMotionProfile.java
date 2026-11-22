package org.firstinspires.ftc.teamcode.util;

/** Rest-to-rest velocity profile with a triangular velocity curve on short moves. */
public final class TrapezoidalMotionProfile implements MotionProfile {
    private final double startPosition;
    private final double endPosition;
    private final double direction;
    private final double maxAcceleration;
    private final double peakVelocity;
    private final double accelerationTime;
    private final double cruiseEndTime;
    private final double duration;

    public TrapezoidalMotionProfile(
            double startPosition, double endPosition, double maxVelocity, double maxAcceleration) {
        double displacement = endPosition - startPosition;
        if (!Double.isFinite(displacement) || !Double.isFinite(maxVelocity) || maxVelocity <= 0.0
                || !Double.isFinite(maxAcceleration) || maxAcceleration <= 0.0) {
            throw new IllegalArgumentException("Motion requires finite positions and positive finite limits");
        }
        this.startPosition = startPosition;
        this.endPosition = endPosition;
        this.direction = Math.signum(displacement);
        this.maxAcceleration = maxAcceleration;

        double distance = Math.abs(displacement);
        peakVelocity = Math.min(maxVelocity, Math.sqrt(distance) * Math.sqrt(maxAcceleration));
        accelerationTime = peakVelocity / maxAcceleration;
        // Total ramp distance is peakVelocity * accelerationTime for both ramps combined.
        cruiseEndTime = distance == 0.0 ? 0.0 : Math.max(accelerationTime, distance / peakVelocity);
        duration = cruiseEndTime + accelerationTime;
        if (!Double.isFinite(duration)
                || (distance > 0.0 && (accelerationTime == 0.0 || duration <= cruiseEndTime))) {
            throw new IllegalArgumentException("Motion duration is outside the representable range");
        }
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
        if (elapsedSeconds < 0.0) {
            return new State(startPosition, 0.0, 0.0);
        }
        if (elapsedSeconds >= duration) {
            return new State(endPosition, 0.0, 0.0);
        }
        if (elapsedSeconds < accelerationTime) {
            double velocity = direction * maxAcceleration * elapsedSeconds;
            return new State(startPosition + 0.5 * velocity * elapsedSeconds,
                    velocity, direction * maxAcceleration);
        }
        if (elapsedSeconds < cruiseEndTime) {
            double velocity = direction * peakVelocity;
            double distance = velocity * (elapsedSeconds - 0.5 * accelerationTime);
            return new State(startPosition + distance, velocity, 0.0);
        }
        double remainingSeconds = duration - elapsedSeconds;
        double velocity = direction * maxAcceleration * remainingSeconds;
        return new State(endPosition - 0.5 * velocity * remainingSeconds,
                velocity, -direction * maxAcceleration);
    }
}
