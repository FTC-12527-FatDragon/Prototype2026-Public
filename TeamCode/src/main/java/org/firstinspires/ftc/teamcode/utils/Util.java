package org.firstinspires.ftc.teamcode.utils;

import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.subsystems.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.subsystems.drive.MecanumDriveOTOS;
import org.opencv.core.CvType;
import org.opencv.core.Mat;

public class Util {
    public static Pose Pose2DToPose(Pose2D pose2D) {
        return new Pose(pose2D.getX(DriveConstants.distanceUnit),
                pose2D.getY(DriveConstants.distanceUnit),
                pose2D.getHeading(DriveConstants.angleUnit));
    }

    public static boolean epsilonEqual(double a, double b, double epsilon) {
        return Math.abs(a - b) <= epsilon;
    }

    public static SparkFunOTOS.Pose2D visionPoseToOTOSPose(Pose3D pose3D) {
        return new SparkFunOTOS.Pose2D(Units.metersToInches(pose3D.getPosition().y) + 72,
                -Units.metersToInches(pose3D.getPosition().x) + 72,
                pose3D.getOrientation().getYaw() / 180 * Math.PI - Math.PI / 2);
    }

    public static Pose2D visionPoseToDWPose(Pose3D pose3D) {
        return new Pose2D(DriveConstants.distanceUnit, Units.metersToInches(pose3D.getPosition().y) + 72,
                -Units.metersToInches(pose3D.getPosition().x) + 72, DriveConstants.angleUnit,
                pose3D.getOrientation().getYaw() / 180 * Math.PI - Math.PI / 2);
    }

    public static double poseDistance(Pose2D poseA, Pose2D poseB) {
        return Math.sqrt(Math.pow(poseA.getX(DriveConstants.distanceUnit) - poseB.getX(DriveConstants.distanceUnit), 2)
                + Math.pow(poseA.getY(DriveConstants.distanceUnit) - poseB.getY(DriveConstants.distanceUnit), 2));
    }
}
