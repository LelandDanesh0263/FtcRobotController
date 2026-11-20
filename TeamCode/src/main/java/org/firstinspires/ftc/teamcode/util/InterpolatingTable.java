package org.firstinspires.ftc.teamcode.util;

import java.util.Map;
import java.util.TreeMap;

public class InterpolatingTable {
    private final TreeMap<Double, Double> table = new TreeMap<>();

    // Add a known data point (e.g., distance -> turret/hood angle or motor power)
    public void put(double key, double value) {
        table.put(key, value);
    }

    // Smoothly interpolates between the two closest data points
    public double get(double key) {
        if (table.isEmpty()) return 0.0;
        if (table.containsKey(key)) return table.get(key);

        Map.Entry<Double, Double> lower = table.lowerEntry(key);
        Map.Entry<Double, Double> upper = table.higherEntry(key);

        if (lower == null) return upper.getValue();
        if (upper == null) return lower.getValue();

        double t = (key - lower.getKey()) / (upper.getKey() - lower.getKey());
        return MathUtil.lerp(lower.getValue(), upper.getValue(), t);
    }

    public void clear() {
        table.clear();
    }
}
