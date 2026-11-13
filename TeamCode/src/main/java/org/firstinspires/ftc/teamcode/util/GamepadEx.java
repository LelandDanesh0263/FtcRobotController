package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.Gamepad;

public class GamepadEx {
    private final Gamepad gamepad;

    private boolean prevA, prevB, prevX, prevY;
    private boolean prevLeftBumper, prevRightBumper;
    private boolean prevDpadUp, prevDpadDown, prevDpadLeft, prevDpadRight;

    private boolean currA, currB, currX, currY;
    private boolean currLeftBumper, currRightBumper;
    private boolean currDpadUp, currDpadDown, currDpadLeft, currDpadRight;

    public GamepadEx(Gamepad gamepad) {
        this.gamepad = gamepad;
    }

    // Call once at the start of every loop
    public void update() {
        prevA = currA; prevB = currB; prevX = currX; prevY = currY;
        prevLeftBumper = currLeftBumper; prevRightBumper = currRightBumper;
        prevDpadUp = currDpadUp; prevDpadDown = currDpadDown;
        prevDpadLeft = currDpadLeft; prevDpadRight = currDpadRight;

        currA = gamepad.a; currB = gamepad.b; currX = gamepad.x; currY = gamepad.y;
        currLeftBumper = gamepad.left_bumper; currRightBumper = gamepad.right_bumper;
        currDpadUp = gamepad.dpad_up; currDpadDown = gamepad.dpad_down;
        currDpadLeft = gamepad.dpad_left; currDpadRight = gamepad.dpad_right;
    }

    public boolean aJustPressed() { return currA && !prevA; }
    public boolean bJustPressed() { return currB && !prevB; }
    public boolean xJustPressed() { return currX && !prevX; }
    public boolean yJustPressed() { return currY && !prevY; }

    public boolean leftBumperJustPressed() { return currLeftBumper && !prevLeftBumper; }
    public boolean rightBumperJustPressed() { return currRightBumper && !prevRightBumper; }

    public boolean dpadUpJustPressed() { return currDpadUp && !prevDpadUp; }
    public boolean dpadDownJustPressed() { return currDpadDown && !prevDpadDown; }
    public boolean dpadLeftJustPressed() { return currDpadLeft && !prevDpadLeft; }
    public boolean dpadRightJustPressed() { return currDpadRight && !prevDpadRight; }

    // Direct access to sticks and triggers
    public double getLeftX() { return gamepad.left_stick_x; }
    public double getLeftY() { return -gamepad.left_stick_y; } // Inverted so pushing up is positive
    public double getRightX() { return gamepad.right_stick_x; }
    public double getRightY() { return -gamepad.right_stick_y; }
    public double getLeftTrigger() { return gamepad.left_trigger; }
    public double getRightTrigger() { return gamepad.right_trigger; }
}
