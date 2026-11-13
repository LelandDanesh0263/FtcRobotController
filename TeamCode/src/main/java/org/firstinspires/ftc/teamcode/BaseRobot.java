package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.Subsystem;
import org.firstinspires.ftc.teamcode.util.DashboardManager;
import org.firstinspires.ftc.teamcode.util.GamepadEx;
import org.firstinspires.ftc.teamcode.util.LoopOptimizer;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class BaseRobot {
    public final LoopOptimizer loopOptimizer;
    public final DashboardManager dashboard;
    public final GamepadEx driver1;
    public final GamepadEx driver2;

    private final List<Subsystem> subsystems = new ArrayList<>();
    // Future subsystems (e.g., Lift, Intake, Drivetrain) will be added right here!

    public BaseRobot(HardwareMap hardwareMap, Telemetry telemetry, Gamepad g1, Gamepad g2) {
        loopOptimizer = new LoopOptimizer(hardwareMap);
        dashboard = new DashboardManager(telemetry);
        driver1 = new GamepadEx(g1);
        driver2 = new GamepadEx(g2);
    }

    // Call once at the start of every TeleOp / Auto loop
    public void update() {
        loopOptimizer.clearBulkCache();
        driver1.update();
        driver2.update();

        // Future subsystem .periodic() updates will run right here!

        dashboard.update();
    }

    public void stopAll() {
        for (Subsystem subsystem : subsystems) {
            subsystem.stop();
        }
    }
}
