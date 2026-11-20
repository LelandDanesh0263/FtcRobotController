package org.firstinspires.ftc.teamcode.util;

public class TurretMath {

    // Calculates straight-line distance from robot (x, y) to any field target (targetX, targetY)
    public static double getDistanceToTarget(double robotX, double robotY, double targetX, double targetY) {
        return Math.hypot(targetX - robotX, targetY - robotY);
    }

    // Calculates the exact robot-relative turret angle (in radians) needed to aim at a field coordinate
    public static double getTargetTurretAngle(
            double robotX, double robotY, double robotHeadingRadians,
            double targetX, double targetY) {

        double fieldAngleToTarget = Math.atan2(targetY - robotY, targetX - robotX);
        double robotRelativeAngle = fieldAngleToTarget - robotHeadingRadians;
        return MathUtil.wrapAngleRadians(robotRelativeAngle);
    }

    // Clamps target turret angle within physical cable/hard-stop limits (in radians)
    public static double clampToTurretLimits(double targetAngleRad, double minLimitRad, double maxLimitRad) {
        return MathUtil.clamp(targetAngleRad, minLimitRad, maxLimitRad);
    }
}
