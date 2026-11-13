package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.BaseRobot;

@TeleOp(name = "Main TeleOp", group = "Competition")
public class MainTeleOp extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        BaseRobot robot = new BaseRobot(hardwareMap, telemetry, gamepad1, gamepad2);
        ElapsedTime loopTimer = new ElapsedTime();

        robot.dashboard.getTelemetry().addLine("Status: Initialized & Ready!");
        robot.dashboard.update();

        waitForStart();
        loopTimer.reset();

        while (opModeIsActive()) {
            // 1. Update bulk cache, gamepads, all subsystems, and Dashboard
            robot.update();

            // 2. Measure and graph loop speed (Hz)
            double loopSeconds = loopTimer.seconds();
            loopTimer.reset();
            double loopHz = (loopSeconds > 0) ? (1.0 / loopSeconds) : 0;

            robot.dashboard.getTelemetry().addData("Loop Speed (Hz)", "%.1f", loopHz);
            robot.dashboard.graph("Loop Hz", loopHz);

            // 3. Future driver controls will go right here!
            // Example: if (robot.driver1.aJustPressed()) { ... }
        }

        // Safely stop everything when Stop is pressed
        robot.stopAll();
    }
}
