package org.firstinspires.ftc.teamcode.subsystems.drive;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Config
public class DriveConstants {
    public static double xPoseDW = 0, yPoseDW = 0;
    public static double xPoseOTOS = 0.0, yPoseOTOS = -52.55, headingPoseOTOS = -Math.PI;
    public static double strafingBalance = 1.1;
    public static double headingEpsilon = 0.1;
    public static DistanceUnit distanceUnit = DistanceUnit.MM;
    public static AngleUnit angleUnit = AngleUnit.RADIANS;
}
