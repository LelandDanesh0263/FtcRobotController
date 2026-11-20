package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "Basic TeleOp", group = "Competition")
public class BasicTeleOp extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        DcMotor frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        DcMotor backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        DcMotor backRight = hardwareMap.get(DcMotor.class, "backRight");

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);

        DcMotor[] driveMotors = {frontLeft, frontRight, backLeft, backRight};
        try {
            for (DcMotor motor : driveMotors) {
                motor.setPower(0);
                motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            }

            telemetry.addLine("Ready");
            telemetry.update();
            waitForStart();

            while (opModeIsActive()) {
                double forward = -gamepad1.left_stick_y;
                double strafe = gamepad1.left_stick_x;
                double turn = gamepad1.right_stick_x;
                double scale = Math.max(1.0, Math.abs(forward) + Math.abs(strafe) + Math.abs(turn));

                frontLeft.setPower((forward + strafe + turn) / scale);
                frontRight.setPower((forward - strafe - turn) / scale);
                backLeft.setPower((forward - strafe + turn) / scale);
                backRight.setPower((forward + strafe - turn) / scale);
            }
        } finally {
            for (DcMotor motor : driveMotors) {
                motor.setPower(0);
            }
        }
    }
}
