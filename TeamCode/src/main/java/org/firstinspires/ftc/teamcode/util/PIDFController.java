package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.util.ElapsedTime;

public class PIDFController {
    private final PIDFParams params;
    private double integralSum = 0;
    private double lastError = 0;
    private double lastTarget = 0;
    private double lastPosition = 0;
    private double integralLimit = Double.POSITIVE_INFINITY;
    private final ElapsedTime timer = new ElapsedTime();
    private boolean firstRun = true;

    public PIDFController(PIDFParams params) {
        this.params = params;
    }

    public double calculate(double target, double current) {
        double dt = timer.seconds();
        timer.reset();
        return calculate(target, current, dt, params.kF);
    }

    /** Uses explicit elapsed seconds and per-tick feedforward. Pick one timing mode per instance. */
    public double calculate(double target, double current, double dt, double feedforward) {
        if (!Double.isFinite(dt) || dt < 0.0) {
            throw new IllegalArgumentException("Elapsed time must be finite and nonnegative");
        }
        lastTarget = target;
        lastPosition = current;
        double error = target - current;

        if (firstRun) {
            lastError = error;
            firstRun = false;
            return (params.kP * error) + feedforward;
        }

        // Prevent integral windup when close to or at target
        if (Math.abs(error) < params.tolerance) {
            integralSum = 0;
        } else {
            integralSum = MathUtil.clamp(integralSum + error * dt, -integralLimit, integralLimit);
        }

        double derivative = (dt > 0) ? (error - lastError) / dt : 0;
        lastError = error;

        return (params.kP * error)
                + (params.kI * integralSum)
                + (params.kD * derivative)
                + feedforward;
    }

    /** Bounds accumulated error in error-seconds. Set to zero to disable accumulation. */
    public void setIntegralLimit(double limit) {
        if (Double.isNaN(limit) || limit < 0.0) {
            throw new IllegalArgumentException("Integral limit must be nonnegative");
        }
        integralLimit = limit;
        integralSum = MathUtil.clamp(integralSum, -limit, limit);
    }

    public boolean atTarget() {
        return Math.abs(lastTarget - lastPosition) <= params.tolerance;
    }

    public void reset() {
        integralSum = 0;
        lastError = 0;
        firstRun = true;
    }

    // Call this in any tuning OpMode to graph Target vs. Actual live in FTC Dashboard
    public void sendTelemetry(String mechanismName) {
        TelemetryPacket packet = new TelemetryPacket();
        packet.put(mechanismName + " Target", lastTarget);
        packet.put(mechanismName + " Actual", lastPosition);
        packet.put(mechanismName + " Error", lastTarget - lastPosition);
        FtcDashboard.getInstance().sendTelemetryPacket(packet);
    }
}
