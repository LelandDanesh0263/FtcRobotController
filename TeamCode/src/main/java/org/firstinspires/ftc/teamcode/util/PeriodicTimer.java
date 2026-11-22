package org.firstinspires.ftc.teamcode.util;

/** Nonblocking cadence for slow tasks polled from the OpMode loop. */
public final class PeriodicTimer {
    private final long intervalNanos;
    private boolean started;
    private long lastDueNanos;

    /** A zero interval makes every poll due. */
    public PeriodicTimer(long intervalNanos) {
        if (intervalNanos < 0) {
            throw new IllegalArgumentException("Interval must be nonnegative");
        }
        this.intervalNanos = intervalNanos;
    }

    /**
     * Returns true on the first poll or after the interval since the last true result.
     * Supply timestamps from one monotonic clock such as System.nanoTime().
     * A late poll starts a new interval, so missed periods do not queue up.
     */
    public boolean isDue(long nowNanos) {
        if (started && intervalNanos != 0 && nowNanos - lastDueNanos < intervalNanos) {
            return false;
        }
        started = true;
        lastDueNanos = nowNanos;
        return true;
    }

    /** Makes the next poll due immediately. */
    public void reset() {
        started = false;
    }
}
