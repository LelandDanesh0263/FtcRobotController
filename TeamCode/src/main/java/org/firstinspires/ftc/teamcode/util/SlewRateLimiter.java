package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.util.ElapsedTime;

public class SlewRateLimiter {
    private final double positiveRateLimit;
    private final double negativeRateLimit;
    private double prevVal;
    private final ElapsedTime timer = new ElapsedTime();

    public SlewRateLimiter(double positiveRateLimit, double negativeRateLimit, double initialValue) {
        this.positiveRateLimit = positiveRateLimit;
        this.negativeRateLimit = Math.abs(negativeRateLimit);
        this.prevVal = initialValue;
        timer.reset();
    }

    public SlewRateLimiter(double rateLimit) {
        this(rateLimit, rateLimit, 0.0);
    }

    public double calculate(double input) {
        double dt = timer.seconds();
        timer.reset();

        double diff = input - prevVal;
        double maxUp = positiveRateLimit * dt;
        double maxDown = -negativeRateLimit * dt;

        prevVal += MathUtil.clamp(diff, maxDown, maxUp);
        return prevVal;
    }

    public void reset(double value) {
        prevVal = value;
        timer.reset();
    }
}
