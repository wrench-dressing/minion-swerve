package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

public class Vector {
    public double x;
    public double y;

    public Vector(double x, double y) {
        this.x = x;
        this.y = y;
    }

    @SuppressWarnings("SuspiciousNameCombination")
    public Vector reversed() {
        double prevX = x;
        x = y;
        y = prevX;
        return this;
    }

    public double angle() {
        return Math.toDegrees(Math.atan2(y, x));
    }

    public double magnitude() {
        return Utils.clamp(0, 1, Math.hypot(x, y));
    }

    public Vector deadbanded(double deadband) {
        if (magnitude() < deadband) {
            x = 0;
            y = 0;
        }

        return this;
    }

    public boolean isZeroed() {
        return x == 0 && y == 0;
    }

    @NonNull
    @Override
    public String toString() {
        return "Vector{ " + x + "  " + y + "}";
    }
}
