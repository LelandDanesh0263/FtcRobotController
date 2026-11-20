package org.firstinspires.ftc.teamcode.util;

public class LowPassFilter {
    private double alpha; // Smoothing factor between 0.0 (heavy smoothing) and 1.0 (no smoothing)
    private double estimate = 0.0;
    private boolean initialized = false;

    public LowPassFilter(double alpha) {
        this.alpha = MathUtil.clamp(alpha, 0.0, 1.0);
    }

    public void setAlpha(double alpha) {
        this.alpha = MathUtil.clamp(alpha, 0.0, 1.0);
    }

    public double calculate(double measurement) {
        if (!initialized) {
            estimate = measurement;
            initialized = true;
            return estimate;
        }
        estimate = (alpha * measurement) + ((1.0 - alpha) * estimate);
        return estimate;
    }

    public void reset() {
        initialized = false;
        estimate = 0.0;
    }
}
