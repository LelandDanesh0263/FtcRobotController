package org.firstinspires.ftc.teamcode.util;

/** Hysteretic load shedding for discretionary electrical loads. */
public final class BrownoutGuard {
    private final double tripVoltage;
    private final double recoverVoltage;
    private final long recoveryDwellNanos;
    private final double shedScale;
    private boolean shedding;
    private boolean recovering;
    private long recoveryStartNanos;

    public BrownoutGuard(
            double tripVoltage, double recoverVoltage, long recoveryDwellNanos, double shedScale) {
        if (!Double.isFinite(tripVoltage) || !Double.isFinite(recoverVoltage)
                || tripVoltage <= 0.0 || recoverVoltage <= tripVoltage) {
            throw new IllegalArgumentException("Recovery voltage must exceed the positive trip voltage");
        }
        if (recoveryDwellNanos < 0 || !Double.isFinite(shedScale) || shedScale < 0.0 || shedScale > 1.0) {
            throw new IllegalArgumentException("Use a nonnegative recovery time and scale in [0, 1]");
        }
        this.tripVoltage = tripVoltage;
        this.recoverVoltage = recoverVoltage;
        this.recoveryDwellNanos = recoveryDwellNanos;
        this.shedScale = shedScale;
    }

    /**
     * Enter shedding at or below the trip voltage or when voltage is unavailable. Leave
     * shedding after voltage has stayed at or above the recovery threshold for the dwell.
     * Call every tick with a fresh or staleness-checked voltage and monotonic time.
     */
    public boolean update(double voltage, long nowNanos) {
        if (!Double.isFinite(voltage) || voltage <= tripVoltage) {
            shedding = true;
            recovering = false;
        } else if (shedding && voltage >= recoverVoltage) {
            if (!recovering) {
                recovering = true;
                recoveryStartNanos = nowNanos;
            }
            if (nowNanos - recoveryStartNanos >= recoveryDwellNanos) {
                shedding = false;
                recovering = false;
            }
        } else {
            recovering = false;
        }
        return shedding;
    }

    /** Multiplier for optional loads such as intake. Leave essential controls independent. */
    public double getDiscretionaryScale() {
        return shedding ? shedScale : 1.0;
    }

    public void reset() {
        shedding = false;
        recovering = false;
    }
}
