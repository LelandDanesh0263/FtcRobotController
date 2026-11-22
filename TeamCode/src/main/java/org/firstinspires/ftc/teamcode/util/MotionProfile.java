package org.firstinspires.ftc.teamcode.util;

/** Time-based setpoints for moving a single mechanism axis. */
public interface MotionProfile {
    double durationSeconds();

    /** Samples time since the move started. Times outside the move hold the nearest endpoint. */
    State sample(double elapsedSeconds);

    /** Position, velocity, and acceleration in consistent units, with time measured in seconds. */
    final class State {
        public final double position;
        public final double velocity;
        public final double acceleration;

        public State(double position, double velocity, double acceleration) {
            this.position = position;
            this.velocity = velocity;
            this.acceleration = acceleration;
        }
    }
}
