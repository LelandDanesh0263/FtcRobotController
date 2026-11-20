package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.util.ElapsedTime;

public class Debouncer {
    private final double debounceSeconds;
    private final ElapsedTime timer = new ElapsedTime();
    private boolean stableState = false;
    private boolean lastRawState = false;

    public Debouncer(double debounceSeconds) {
        this.debounceSeconds = debounceSeconds;
    }

    public boolean calculate(boolean rawInput) {
        if (rawInput != lastRawState) {
            timer.reset();
            lastRawState = rawInput;
        }

        if (timer.seconds() >= debounceSeconds) {
            stableState = rawInput;
        }

        return stableState;
    }

    public void reset() {
        stableState = false;
        lastRawState = false;
        timer.reset();
    }
}
