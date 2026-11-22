package org.firstinspires.ftc.teamcode.util;

/** Detects a sustained high-current, low-speed condition while a motor is being driven. */
public final class StallDetector {
    private final double minPower;
    private final double minCurrentAmps;
    private final double maxSpeed;
    private final long stallDwellNanos;
    private boolean suspect;
    private long suspectSinceNanos;

    public StallDetector(double minPower, double minCurrentAmps, double maxSpeed, long stallDwellNanos) {
        if (!Double.isFinite(minPower) || minPower <= 0.0 || minPower > 1.0
                || !Double.isFinite(minCurrentAmps) || minCurrentAmps <= 0.0
                || !Double.isFinite(maxSpeed) || maxSpeed < 0.0 || stallDwellNanos < 0) {
            throw new IllegalArgumentException("Invalid stall thresholds");
        }
        this.minPower = minPower;
        this.minCurrentAmps = minCurrentAmps;
        this.maxSpeed = maxSpeed;
        this.stallDwellNanos = stallDwellNanos;
    }

    /**
     * Supply the applied power, measured amps, velocity in the configured speed units per
     * second, and a monotonic timestamp. Forward and reverse are treated alike. A healthy or
     * missing sample clears the dwell. The caller decides if and how to stop or reverse.
     */
    public boolean update(double appliedPower, double currentAmps, double velocity, long nowNanos) {
        boolean stallSample = Double.isFinite(appliedPower) && Math.abs(appliedPower) >= minPower
                && Double.isFinite(currentAmps) && currentAmps >= minCurrentAmps
                && Double.isFinite(velocity) && Math.abs(velocity) <= maxSpeed;
        if (!stallSample) {
            reset();
            return false;
        }
        if (!suspect) {
            suspect = true;
            suspectSinceNanos = nowNanos;
        }
        return nowNanos - suspectSinceNanos >= stallDwellNanos;
    }

    public void reset() {
        suspect = false;
    }
}
