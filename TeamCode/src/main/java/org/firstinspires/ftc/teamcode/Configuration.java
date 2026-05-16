package org.firstinspires.ftc.teamcode;

public class Configuration {
    public static final double maxSpeed = 0.3;
    public static final double angularMaxSpeed = 360;
    public static final double maxAccelRps = maxSpeed * 5;

    // robot wont drive if the modules arent within this of the desired position
    public static final double rotToleranceToDriveDegrees = 45;
    public static final double rotToleranceToDrive = rotToleranceToDriveDegrees / 360.0;

    public static final double rotationGain = 3;

    public static final String FRONT_TOP_MOTOR = "fronttop";
    public static final String FRONT_BOTTOM_MOTOR = "frontbot";
    public static final String BACK_TOP_MOTOR = "backtop";
    public static final String BACK_BOTTOM_MOTOR = "backbot";
}
