package org.firstinspires.ftc.teamcode.subsystems.vision;

import com.qualcomm.hardware.limelightvision.LLResultTypes;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.utils.Util;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;

public class VisionConstants {
    public enum AprilTags{
        NUM20(371.736, 3266.895, 747.827, 0.952613, 0, 0),
        NUM21(1785.144, 3604.031, 459.855, 0, 0, 0),
        NUM22(1785.144, 3604.031, 459.855, 0, 0, 0),
        NUM23(1785.144, 3604.031, 459.855, 0, 0, 0),
        NUM24(3198.551, 3266.895, 747.827, -0.952613, 0, 0);

        public final double tagX, tagY, tagZ, tagYaw, tagRoll, tagPitch;

        AprilTags(double tagX, double tagY, double tagZ, double tagYaw, double tagRoll, double tagPitch) {
            this.tagX = tagX;
            this.tagY = tagY;
            this.tagZ = tagZ;
            this.tagYaw = tagYaw;
            this.tagRoll = tagRoll;
            this.tagPitch = tagPitch;
        }
    }

    private static AprilTags getTag(LLResultTypes.FiducialResult fiducialResult) {
        switch (fiducialResult.getFiducialId()){
            case 20:
                return AprilTags.NUM20;
            case 21:
                return AprilTags.NUM21;
            case 22:
                return AprilTags.NUM22;
            case 23:
                return AprilTags.NUM23;
            case 24:
                return AprilTags.NUM24;
            default:
                return null;
        }
    }

    /**
     * @param fiducialResult limelight返回的result（从fiducialResult获取）
     * @return 相机相对于场地左下角的姿态字符串（X/Y/Z为坐标，Pitch/Roll/Yaw为姿态角，单位与输入一致）
     */
    public static Pose3D getCameraFieldPose(LLResultTypes.FiducialResult fiducialResult) {
        VisionConstants.AprilTags tagsPose = getTag(fiducialResult);
        if (tagsPose == null) return null;
        Pose3D cameraPoseTargetSpace = fiducialResult.getCameraPoseTargetSpace();
        Pose3D tagPoseInField = new Pose3D(new Position(DistanceUnit.MM, tagsPose.tagX,
                tagsPose.tagY, tagsPose.tagZ, cameraPoseTargetSpace.getPosition().acquisitionTime),
                new YawPitchRollAngles(AngleUnit.RADIANS, tagsPose.tagYaw, tagsPose.tagPitch,
                        tagsPose.tagRoll, cameraPoseTargetSpace.getOrientation().getAcquisitionTime()));

        Position cameraPose = cameraPoseTargetSpace.getPosition().toUnit(DistanceUnit.MM);
        YawPitchRollAngles cameraAngles = cameraPoseTargetSpace.getOrientation();
        Pose3D cameraPoseTagFrame = new Pose3D(new Position(DistanceUnit.MM, cameraPose.x,
                cameraPose.y, cameraPose.z, cameraPose.acquisitionTime), new YawPitchRollAngles(
                        AngleUnit.RADIANS, cameraAngles.getYaw(), cameraAngles.getPitch(),
                cameraAngles.getRoll(), cameraAngles.getAcquisitionTime()));

        //return Util.compose(tagPoseInField, cameraPoseTagFrame);
        return cameraPoseTargetSpace;
    }
}