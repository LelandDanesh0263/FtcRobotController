package org.firstinspires.ftc.teamcode.util;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.BaseRobot;

/** Add, remove, or reorder lines here to change the Driver Hub display. */
public final class RobotTelemetry {
    private RobotTelemetry() {
    }

    public static void showReady(Telemetry telemetry) {
        telemetry.addData("Status", "Initialized & ready");
    }

    public static void report(Telemetry telemetry, BaseRobot robot, double loopSeconds) {
        double loopHz = loopSeconds > 0 ? 1.0 / loopSeconds : 0;
        telemetry.addData("Loop speed (Hz)", "%.1f", loopHz);
        telemetry.addData("Battery (V)", "%.2f", robot.loopOptimizer.getBatteryVoltage());
        telemetry.addData("Camera streaming", robot.vision.isCameraInitialized());
        telemetry.addData("Vision cameras", robot.vision.getCameraStatuses());
        telemetry.addData("Fresh vision observations", robot.vision.getObservations().size());
        telemetry.addData("Vision errors", robot.vision.getErrors());
        telemetry.addData("Closest AprilTag with pose", robot.vision.getClosestAprilTag());
        // telemetry.addData("Color blobs", robot.vision.getDetectedBlobs().size());
    }
}
