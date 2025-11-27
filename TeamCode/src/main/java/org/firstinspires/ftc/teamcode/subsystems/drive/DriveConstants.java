package org.firstinspires.ftc.teamcode.subsystems.drive;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.utils.Units;

@Config
public class DriveConstants {
    public static String leftFrontMotorName = "leftFrontMotor";
    public static String leftBackMotorName = "leftBackMotor";
    public static String rightFrontMotorName = "rightFrontMotor";
    public static String rightBackMotorName = "rightBackMotor";

    public static double xPoseDW = 0, yPoseDW = 0;
    public static double xPoseOTOS = Units.mmToInches(-190.9), yPoseOTOS = 0, headingPoseOTOS = Math.PI / 2;

    public static double strafingBalance = 1.1;
    public static double headingEpsilon = 0.1;
    public static DistanceUnit distanceUnit = DistanceUnit.INCH;
    public static AngleUnit angleUnit = AngleUnit.RADIANS;

    public static double linearScalar = 1.00026, angularScalar = 0.99414;
    public static double forwardVelocity = 63.966, strafeVelocity = 26.744;
    public static double forwardAcceleration = -32.6419, strafeAcceleration = -95.1316;

    public static double kP_xy = 0.02;
    public static double kP_h = -0.8;
}
