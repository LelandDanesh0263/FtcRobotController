package org.firstinspires.ftc.teamcode.util;

import java.util.Objects;
import java.util.function.DoubleConsumer;

/** Suppresses small changes in commands sent to a single actuator on the OpMode thread. */
public final class CachedOutput {
    private final DoubleConsumer writer;
    private final double epsilon;
    private boolean hasSentValue;
    private double lastSentValue;

    /** Epsilon uses the same units as the actuator command. */
    public CachedOutput(DoubleConsumer writer, double epsilon) {
        this.writer = Objects.requireNonNull(writer, "writer");
        if (!Double.isFinite(epsilon) || epsilon < 0.0) {
            throw new IllegalArgumentException("Epsilon must be finite and nonnegative");
        }
        this.epsilon = epsilon;
    }

    /**
     * Sends the first command, changes larger than epsilon relative to the last write,
     * and any transition to zero. Returns true when a write occurs.
     */
    public boolean write(double command) {
        requireFiniteCommand(command);
        if (hasSentValue && Math.abs(command - lastSentValue) <= epsilon
                && (command != 0.0 || lastSentValue == 0.0)) {
            return false;
        }
        send(command);
        return true;
    }

    /** Sends a command regardless of the cache, for example zero during motor shutdown. */
    public void forceWrite(double command) {
        requireFiniteCommand(command);
        send(command);
    }

    /** Call after an external write or a device reset so the next command is sent. */
    public void invalidate() {
        hasSentValue = false;
    }

    private void send(double command) {
        // A failed hardware write can leave the actual device state unknown.
        invalidate();
        writer.accept(command);
        lastSentValue = command;
        hasSentValue = true;
    }

    private static void requireFiniteCommand(double command) {
        if (!Double.isFinite(command)) {
            throw new IllegalArgumentException("Actuator command must be finite");
        }
    }
}
