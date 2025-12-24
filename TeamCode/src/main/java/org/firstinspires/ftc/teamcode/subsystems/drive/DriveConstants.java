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

    public static double xPoseDW = Units.mmToInches(-171.86), yPoseDW = Units.mmToInches(-118.69);

    public static double strafingBalance = 1.1;
    public static double headingEpsilon = 0.1;
    public static DistanceUnit distanceUnit = DistanceUnit.INCH;
    public static AngleUnit angleUnit = AngleUnit.RADIANS;

    public static double linearScalar = 1.00026, angularScalar = 0.99414;
    public static double forwardVelocity = 81.938, strafeVelocity = 63.501;
    public static double forwardAcceleration = -33.933, strafeAcceleration = -53.729;

    public static double kP_brakeXY = 0.02;
    public static double kP_brakeH = -0.8;
    public static double kP_alignH = -1.2;
    public static double kI_alignH = 0;
    public static double kD_alignH = -0.115;
    public static double kP_followXY = 0;
    public static double kP_followH = 0;

    public static double xNearPoseRed = 144, yNearPoseRed = 144;
    public static double xNearPoseBlue = 4, yNearPoseBlue = 144;
    public static double xFarPoseRed = 142, yFarPoseRed = 144;
    public static double xFarPoseBlue = 5, yFarPoseBlue = 144;

    public static double nearGoalDistance = 0, farGoalDistance = 0;

}
