package org.firstinspires.ftc.teamcode.subsystems.shooter;


import com.acmerobotics.dashboard.config.Config;

@Config
public class ShooterConstants {
    public static String leftShooterName = "leftShooter";
    public static String rightShooterName = "rightShooter";

    public static double shooterEpsilon = 50;

    public static double kP = 1;
    public static double kI = 0;
    public static double kD = 0;
    public static double stopPower = 0;
    public static double slowPower = 0.575;
    public static double fastPower = 0.675;

    /**
     * In Ticks Per Second
     */
    public static double stopVelocity = 0;
    public static double fastVelocity = -1880;
    public static double slowVelocity = -1580;
}
