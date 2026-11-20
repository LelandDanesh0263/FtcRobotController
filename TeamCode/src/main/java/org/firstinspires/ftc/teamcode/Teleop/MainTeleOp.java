package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.BaseRobot;

@TeleOp(name = "Main TeleOp", group = "Competition")
public class MainTeleOp extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        BaseRobot robot = new BaseRobot(hardwareMap, telemetry, gamepad1, gamepad2);
        try {
            robot.showReady();
            waitForStart();

            while (opModeIsActive()) {
                robot.update();

                // Future driver controls will go right here!
                // Example: if (robot.driver1.aJustPressed()) { ... }
            }
        } finally {
            robot.stopAll();
        }
    }
}
