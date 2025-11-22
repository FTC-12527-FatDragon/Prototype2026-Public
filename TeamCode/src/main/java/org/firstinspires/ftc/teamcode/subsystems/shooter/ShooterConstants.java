package org.firstinspires.ftc.teamcode.subsystems.shooter;


import com.acmerobotics.dashboard.config.Config;

@Config
public class ShooterConstants {
    public static String leftShooterName = "leftShooterMotor";
    public static String rightShooterName = "rightShooterMotor";

    public static double shooterEpsilon = 40;

    public static double kP = 0.5;
    public static double kI = 0;
    public static double kD = 0;
    public static double stopPower = 0;
    public static double slowPower = 0.575;
    public static double fastPower = 0.675;

    /**
     * In Ticks Per Second
     */
    public static double stopVelocity = 0;
    public static double fastVelocity = 1520;
    public static double slowVelocity = 1320;
    public static double releaseVelocity = 1000;
}
