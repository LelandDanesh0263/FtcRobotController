package org.firstinspires.ftc.teamcode;

public class RobotState {
    // Field Pose (updated by odometry/localization later)
    public double x = 0.0;
    public double y = 0.0;
    public double headingRadians = 0.0;

    // Velocity Telemetry
    public double vx = 0.0;
    public double vy = 0.0;
    public double omega = 0.0;

    // Shared Game-Piece & Mechanism Flags
    public boolean hasGamePiece = false;
    public boolean isScoringReady = false;
    public String activeSuperstructureState = "IDLE";

    public void resetPose(double x, double y, double headingRadians) {
        this.x = x;
        this.y = y;
        this.headingRadians = headingRadians;
    }
}
