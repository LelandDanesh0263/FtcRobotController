package org.firstinspires.ftc.teamcode.subsystems;

public interface Subsystem {
    // Runs every single loop automatically (for PID updates, telemetry, etc.)
    default void periodic() {}

    // Safety method to stop all motors/servos when an OpMode ends
    default void stop() {}
}
