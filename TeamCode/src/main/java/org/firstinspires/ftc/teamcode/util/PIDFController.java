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
    private final ElapsedTime timer = new ElapsedTime();
    private boolean firstRun = true;

    public PIDFController(PIDFParams params) {
        this.params = params;
    }

    public double calculate(double target, double current) {
        lastTarget = target;
        lastPosition = current;
        double error = target - current;

        if (firstRun) {
            lastError = error;
            timer.reset();
            firstRun = false;
            return (params.kP * error) + params.kF;
        }

        double dt = timer.seconds();
        timer.reset();

        // Prevent integral windup when close to or at target
        if (Math.abs(error) < params.tolerance) {
            integralSum = 0;
        } else {
            integralSum += error * dt;
        }

        double derivative = (dt > 0) ? (error - lastError) / dt : 0;
        lastError = error;

        return (params.kP * error)
                + (params.kI * integralSum)
                + (params.kD * derivative)
                + params.kF;
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
