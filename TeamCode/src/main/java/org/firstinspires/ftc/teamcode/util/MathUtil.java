package org.firstinspires.ftc.teamcode.util;

public class MathUtil {

    // Constrains a value between a minimum and maximum
    public static double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

    // Applies a deadband and rescales the remaining range smoothly from 0.0 to 1.0
    public static double applyDeadband(double value, double deadband) {
        if (Math.abs(value) < deadband) {
            return 0.0;
        }
        return Math.copySign((Math.abs(value) - deadband) / (1.0 - deadband), value);
    }

    // Shapes joystick inputs (exponent = 2.0 for squared curve, 3.0 for cubed)
    public static double shapeInput(double value, double deadband, double exponent) {
        double deadbanded = applyDeadband(value, deadband);
        return Math.copySign(Math.pow(Math.abs(deadbanded), exponent), deadbanded);
    }

    // Wraps any angle in radians to [-PI, PI] (shortest rotation path)
    public static double wrapAngleRadians(double angleRadians) {
        double wrapped = (angleRadians + Math.PI) % (2.0 * Math.PI);
        if (wrapped < 0) {
            wrapped += 2.0 * Math.PI;
        }
        return wrapped - Math.PI;
    }

    // Wraps any angle in degrees to [-180, 180]
    public static double wrapAngleDegrees(double angleDegrees) {
        double wrapped = (angleDegrees + 180.0) % 360.0;
        if (wrapped < 0) {
            wrapped += 360.0;
        }
        return wrapped - 180.0;
    }

    // Linear interpolation between two values
    public static double lerp(double start, double end, double t) {
        return start + t * (end - start);
    }
}
