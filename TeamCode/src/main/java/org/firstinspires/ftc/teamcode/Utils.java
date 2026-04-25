package org.firstinspires.ftc.teamcode;

public class Utils {
    public static double clamp(double min, double max, double val) {
        return Math.min(Math.max(min, val), max);
    }
}
