package org.firstinspires.ftc.teamcode.util;

public class PIDFParams {
    public double kP;
    public double kI;
    public double kD;
    public double kF;
    public double tolerance;

    public PIDFParams(double kP, double kI, double kD, double kF, double tolerance) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kF = kF;
        this.tolerance = tolerance;
    }

    public PIDFParams(double kP, double kI, double kD, double kF) {
        this(kP, kI, kD, kF, 5.0);
    }
}
