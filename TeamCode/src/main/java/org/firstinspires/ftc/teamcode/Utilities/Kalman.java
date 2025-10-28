package org.firstinspires.ftc.teamcode.Utilities;

public class Kalman {
    //process noise
    //increase if too slow
    private double q;
    //measurement noise
    //increase if noisy
    private double r;
    private double p;
    private double k;
    private double x;

    public Kalman(double q, double r, double initialValue) {
        this.q = q;
        this.r = r;
        this.x = initialValue;
        this.p = 1;
    }

    public double update(double measurement) {
        p += q;
        k = p / (p + r);
        x += k * (measurement - x);
        p *= (1 - k);
        return x;
    }
}

