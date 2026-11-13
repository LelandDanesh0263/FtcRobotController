package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class DashboardManager {
    private final FtcDashboard dashboard;
    private final MultipleTelemetry telemetry;
    private TelemetryPacket currentPacket;

    public DashboardManager(Telemetry opModeTelemetry) {
        dashboard = FtcDashboard.getInstance();
        // Mirrors standard telemetry.addData() to BOTH Driver Station and Dashboard
        telemetry = new MultipleTelemetry(opModeTelemetry, dashboard.getTelemetry());
        currentPacket = new TelemetryPacket();
    }

    public MultipleTelemetry getTelemetry() {
        return telemetry;
    }

    // Use this to plot live numbers on FTC Dashboard graphs
    public void graph(String key, double value) {
        currentPacket.put(key, value);
    }

    // Call once at the end of every loop to push all graphs and text
    public void update() {
        telemetry.update();
        dashboard.sendTelemetryPacket(currentPacket);
        currentPacket = new TelemetryPacket();
    }
}
