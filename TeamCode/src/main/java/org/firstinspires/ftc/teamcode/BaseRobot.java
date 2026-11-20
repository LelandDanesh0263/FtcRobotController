package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.Subsystem;
import org.firstinspires.ftc.teamcode.subsystems.Vision;
import org.firstinspires.ftc.teamcode.util.DashboardManager;
import org.firstinspires.ftc.teamcode.util.GamepadEx;
import org.firstinspires.ftc.teamcode.util.LoopOptimizer;
import org.firstinspires.ftc.teamcode.util.RobotTelemetry;

import java.util.ArrayList;
import java.util.List;

public class BaseRobot {
    public final LoopOptimizer loopOptimizer;
    public final DashboardManager dashboard;
    public final GamepadEx driver1;
    public final GamepadEx driver2;
    public final Vision vision;

    private final Telemetry driverTelemetry;
    private final List<Subsystem> subsystems = new ArrayList<>();
    private long previousLoopStartNanos;

    public BaseRobot(HardwareMap hardwareMap, Telemetry telemetry, Gamepad g1, Gamepad g2) {
        driverTelemetry = telemetry;
        loopOptimizer = new LoopOptimizer(hardwareMap);
        dashboard = new DashboardManager(telemetry);
        driver1 = new GamepadEx(g1);
        driver2 = new GamepadEx(g2);

        // Hardware-agnostic vision subsystem
        vision = new Vision(hardwareMap);
        subsystems.add(vision);
    }

    public void showReady() {
        RobotTelemetry.showReady(driverTelemetry);
        driverTelemetry.update();
    }

    // Call once at the start of every TeleOp / Auto loop
    public void update() {
        long loopStartNanos = System.nanoTime();
        double loopSeconds = previousLoopStartNanos == 0
                ? 0 : (loopStartNanos - previousLoopStartNanos) / 1e9;
        previousLoopStartNanos = loopStartNanos;

        loopOptimizer.clearBulkCache();
        driver1.update();
        driver2.update();

        // Update all subsystems
        for (Subsystem subsystem : subsystems) {
            subsystem.periodic();
        }

        RobotTelemetry.report(driverTelemetry, this, loopSeconds);
        dashboard.update();
    }

    public void stopAll() {
        for (Subsystem subsystem : subsystems) {
            subsystem.stop();
        }
    }
}
