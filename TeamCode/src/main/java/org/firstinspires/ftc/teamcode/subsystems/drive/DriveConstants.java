package org.firstinspires.ftc.teamcode.subsystems.drive;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.utils.Units;

@Config
public class DriveConstants {
    public static double xPoseDW = 0, yPoseDW = 0;
    public static double xPoseOTOS = 0, yPoseOTOS = Units.mmToInches(-190.9), headingPoseOTOS = Math.PI;
    public static double strafingBalance = 1.1;
    public static double headingEpsilon = 0.1;
    public static DistanceUnit distanceUnit = DistanceUnit.INCH;
    public static AngleUnit angleUnit = AngleUnit.RADIANS;
}
