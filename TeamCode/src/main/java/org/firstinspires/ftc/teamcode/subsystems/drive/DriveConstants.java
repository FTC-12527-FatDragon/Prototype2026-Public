package org.firstinspires.ftc.teamcode.subsystems.drive;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class DriveConstants {
    public static double xPoseDW = 0, yPoseDW = 0;
    public static double xPoseOTOS = -141.5, yPoseOTOS = -52.55, headingPoseOTOS = -Math.PI;
    public static double strafingBalance = 1.1;
    public static double headingEpsilon = 0.1;
    public static DistanceUnit distanceUnit = DistanceUnit.MM;
    public static AngleUnit angleUnit = AngleUnit.RADIANS;
}
