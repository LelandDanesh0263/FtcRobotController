package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import java.util.List;

public class LoopOptimizer {
    private final List<LynxModule> allHubs;
    private final VoltageSensor batteryVoltageSensor;

    public LoopOptimizer(HardwareMap hardwareMap) {
        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();
    }

    public void clearBulkCache() {
        for (LynxModule hub : allHubs) {
            hub.clearBulkCache();
        }
    }

    public double getVoltageScalar() {
        double voltage = batteryVoltageSensor.getVoltage();
        return (voltage > 0) ? (12.5 / voltage) : 1.0;
    }
}
