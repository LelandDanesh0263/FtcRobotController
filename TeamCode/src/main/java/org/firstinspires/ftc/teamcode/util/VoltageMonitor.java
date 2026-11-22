package org.firstinspires.ftc.teamcode.util;

import java.util.Objects;
import java.util.function.DoubleSupplier;

/** Polls a shared battery voltage source at a configured rate on the owning loop thread. */
public final class VoltageMonitor {
    private final DoubleSupplier readVoltage;
    private final PeriodicTimer sampleTimer;
    private final long maxSampleAgeNanos;
    private double lastValidVoltage = Double.NaN;
    private long lastValidSampleNanos;

    public VoltageMonitor(DoubleSupplier readVoltage, long samplePeriodNanos, long maxSampleAgeNanos) {
        if (maxSampleAgeNanos <= 0 || maxSampleAgeNanos < samplePeriodNanos) {
            throw new IllegalArgumentException("Sample age must be positive and cover the polling period");
        }
        this.readVoltage = Objects.requireNonNull(readVoltage, "readVoltage");
        this.sampleTimer = new PeriodicTimer(samplePeriodNanos);
        this.maxSampleAgeNanos = maxSampleAgeNanos;
    }

    /**
     * Call once per robot tick with a monotonic clock. The reader can return NaN to report
     * an unavailable measurement. A missed sample leaves the previous reading usable until
     * it reaches the maximum age. The returned value can be shared across subsystems.
     */
    public double update(long nowNanos) {
        if (sampleTimer.isDue(nowNanos)) {
            double voltage = readVoltage.getAsDouble();
            if (Double.isFinite(voltage) && voltage > 0.0) {
                lastValidVoltage = voltage;
                lastValidSampleNanos = nowNanos;
            }
        }
        return getVoltage(nowNanos);
    }

    /** Returns the usable cached voltage, or NaN if no current reading is available. */
    public double getVoltage(long nowNanos) {
        long sampleAgeNanos = nowNanos - lastValidSampleNanos;
        if (sampleAgeNanos < 0 || sampleAgeNanos >= maxSampleAgeNanos) {
            return Double.NaN;
        }
        return lastValidVoltage;
    }

    public void reset() {
        sampleTimer.reset();
        lastValidVoltage = Double.NaN;
    }
}
