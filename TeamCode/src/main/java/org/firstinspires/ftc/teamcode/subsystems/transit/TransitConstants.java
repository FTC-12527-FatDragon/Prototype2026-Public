package org.firstinspires.ftc.teamcode.subsystems.transit;

import com.acmerobotics.dashboard.config.Config;

@Config
public class TransitConstants {
    public static String transitName = "transitMotor";
    public static String limitServoName = "limitServo";
    public static String chooseServoName = "chooseServo";

    public static double limitServoClosePos = 0.26;
    public static double limitServoOpenPos = 0.0;

    public static double chooseServoClosePos = 0.6;
    public static double chooseServoOpenPos = 0.25;

    public static double transitIntakePower = 0.4;
    public static double transitShootPower = 1.0;
    public static double transitStopPower = 0.0;
}
