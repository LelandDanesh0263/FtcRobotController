package org.firstinspires.ftc.teamcode.util;

public class GamePieceTracker {
    private final int maxCapacity;
    private int currentCount = 0;

    public GamePieceTracker(int maxCapacity) {
        this.maxCapacity = maxCapacity;
    }

    public void registerIntake() {
        if (currentCount < maxCapacity) {
            currentCount++;
        }
    }

    public void registerScore() {
        if (currentCount > 0) {
            currentCount--;
        }
    }

    public void setCount(int count) {
        this.currentCount = (int) MathUtil.clamp(count, 0, maxCapacity);
    }

    public int getCount() {
        return currentCount;
    }

    public boolean isFull() {
        return currentCount >= maxCapacity;
    }

    public boolean isEmpty() {
        return currentCount <= 0;
    }
}
