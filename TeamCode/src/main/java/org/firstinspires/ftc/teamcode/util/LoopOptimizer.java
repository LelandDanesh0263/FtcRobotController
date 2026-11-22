package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import java.util.Iterator;
import java.util.List;

public class LoopOptimizer {
    private final List<LynxModule> allHubs;
    private final VoltageSensor batteryVoltageSensor;

    public LoopOptimizer(HardwareMap hardwareMap) {
        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
        Iterator<VoltageSensor> voltageSensors = hardwareMap.voltageSensor.iterator();
        batteryVoltageSensor = voltageSensors.hasNext() ? voltageSensors.next() : null;
    }

    public void clearBulkCache() {
        for (LynxModule hub : allHubs) {
            hub.clearBulkCache();
        }
    }

    public double getBatteryVoltage() {
        return batteryVoltageSensor == null ? Double.NaN : batteryVoltageSensor.getVoltage();
    }

    public double getVoltageScalar() {
        double voltage = getBatteryVoltage();
        return (Double.isFinite(voltage) && voltage > 0) ? (12.5 / voltage) : 1.0;
    }

    /** Create once per OpMode, then share the sampled voltage across subsystem updates. */
    public VoltageMonitor createVoltageMonitor(long samplePeriodNanos, long maxSampleAgeNanos) {
        return new VoltageMonitor(this::getBatteryVoltage, samplePeriodNanos, maxSampleAgeNanos);
    }
}
