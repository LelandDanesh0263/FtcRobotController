package org.firstinspires.ftc.teamcode.util;

/** A two-threshold switch for bang-bang output or noisy analog sensor decisions. */
public final class Hysteresis {
    private final double turnOffAt;
    private final double turnOnAt;
    private boolean active;

    public Hysteresis(double turnOffAt, double turnOnAt, boolean initiallyActive) {
        if (!Double.isFinite(turnOffAt) || !Double.isFinite(turnOnAt) || turnOffAt >= turnOnAt) {
            throw new IllegalArgumentException("Thresholds must be finite and ordered");
        }
        this.turnOffAt = turnOffAt;
        this.turnOnAt = turnOnAt;
        active = initiallyActive;
    }

    /** For bang-bang regulation, pass target minus measurement and use the result to choose power. */
    public boolean update(double input) {
        if (input >= turnOnAt) {
            active = true;
        } else if (input <= turnOffAt) {
            active = false;
        }
        return active;
    }

    public void reset(boolean active) {
        this.active = active;
    }
}
