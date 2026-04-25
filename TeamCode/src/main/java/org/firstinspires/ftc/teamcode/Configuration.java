package org.firstinspires.ftc.teamcode;

public class Configuration {
    public static final double maxSpeed = 0.4;
    public static final double turnSpeed = 0.4;

    // robot wont drive if the modules arent within this of the desired position
    public static final double rotToleranceToDriveDegrees = 45;
    public static final double rotToleranceToDrive = rotToleranceToDriveDegrees / 360.0;

    public static final double rotationGain = 5;

    public static final String FRONT_TOP_MOTOR = "fronttop";
    public static final String FRONT_BOTTOM_MOTOR = "frontbot";
    public static final String BACK_TOP_MOTOR = "backtop";
    public static final String BACK_BOTTOM_MOTOR = "backbot";
}
