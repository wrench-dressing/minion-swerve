package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;

public class Ramp {
    private double value;
    private final double maxChange;
    private final ElapsedTime deltaTime;

    public Ramp(double initial, double increasePerSecond) {
        value = initial;
        maxChange = increasePerSecond;

        deltaTime = new ElapsedTime();
        deltaTime.reset();
    }

    public double update(double newValue) {
        double maxChange = deltaTime.seconds() * this.maxChange;
        double desiredChange = newValue - value;
        double actualChange = Utils.clamp(-maxChange, maxChange, desiredChange);

        value += actualChange;
        deltaTime.reset();
        return value;
    }
}
