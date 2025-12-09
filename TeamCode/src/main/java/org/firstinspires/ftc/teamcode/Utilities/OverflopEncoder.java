package org.firstinspires.ftc.teamcode.Utilities;

import com.qualcomm.robotcore.hardware.DcMotorEx;

public class OverflopEncoder {
    private final DcMotorEx motor;

    private int lastRawPosition = 0;
    private long totalPosition = 0;     // Absolute corrected position
    private long deltaPosition = 0;     // Delta for this loop

    private int direction = 1;

    public OverflopEncoder(DcMotorEx motor) {
        this.motor = motor;
        lastRawPosition = motor.getCurrentPosition();
    }

    public void setDirection(int direction) {
        this.direction = direction;
    }

    public void reset() {
        lastRawPosition = motor.getCurrentPosition();
        totalPosition = 0;
        deltaPosition = 0;
    }

    public void update() {
        int raw = motor.getCurrentPosition();
        int diff = raw - lastRawPosition;

        // Detect wrap-around (very large impossible jumps)
        if (diff > 1_000_000) {
            diff -= (long) Integer.MAX_VALUE * 2L + 2L;
        } else if (diff < -1_000_000) {
            diff += (long) Integer.MAX_VALUE * 2L + 2L;
        }

        diff *= direction;

        deltaPosition = diff;          // delta for this update
        totalPosition += diff;         // running total position
        lastRawPosition = raw;
    }

    public long getDelta() {
        return deltaPosition;
    }

    public long getTotal() {
        return totalPosition;
    }
}
