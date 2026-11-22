package org.firstinspires.ftc.teamcode.util;

/** Model-based actuator commands. Choose gains that produce one consistent output unit. */
public final class FeedforwardMath {
    private FeedforwardMath() {}

    /** Uses acceleration for friction direction when starting from rest. */
    public static double calculate(double velocity, double acceleration,
                                   double kS, double kV, double kA, double gravity) {
        double direction = Math.signum(velocity == 0.0 ? acceleration : velocity);
        return kS * direction + kV * velocity + kA * acceleration + gravity;
    }

    /** Gravity compensation for a pivoting arm with its angle measured from horizontal. */
    public static double armGravity(double angleRadians, double kG) {
        return kG * Math.cos(angleRadians);
    }
}
