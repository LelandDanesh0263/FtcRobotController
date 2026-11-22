package org.firstinspires.ftc.teamcode.util;

/** Calculations for software that owns a motor's normalized open-loop output. */
public final class PowerMath {
    private PowerMath() {}

    /**
     * Converts power tuned at nominalVoltage to power at the measured battery voltage.
     * Missing or invalid measurements use the bounded nominal command. Apply at one layer
     * only, since some motor controllers and followers compensate internally.
     */
    public static double voltageCompensatedPower(
            double nominalPower, double nominalVoltage, double measuredVoltage) {
        requirePositive(nominalVoltage, "Nominal voltage");
        double power = boundedPower(nominalPower);
        if (!Double.isFinite(measuredVoltage) || measuredVoltage <= 0.0) {
            return power;
        }
        return MathUtil.clamp(power * nominalVoltage / measuredVoltage, -1.0, 1.0);
    }

    /**
     * Derates a command in proportion to measured current above the limit. This is an
     * output adjustment based on a sample. It cannot enforce a physical current cap.
     * Missing or invalid current measurements use the bounded original command.
     */
    public static double derateForCurrent(double requestedPower, double measuredAmps, double limitAmps) {
        requirePositive(limitAmps, "Current limit");
        double power = boundedPower(requestedPower);
        if (!Double.isFinite(measuredAmps) || measuredAmps <= limitAmps) {
            return power;
        }
        return power * (limitAmps / measuredAmps);
    }

    private static double boundedPower(double power) {
        if (!Double.isFinite(power)) {
            return 0.0;
        }
        return MathUtil.clamp(power, -1.0, 1.0);
    }

    private static void requirePositive(double value, String name) {
        if (!Double.isFinite(value) || value <= 0.0) {
            throw new IllegalArgumentException(name + " must be finite and positive");
        }
    }
}
