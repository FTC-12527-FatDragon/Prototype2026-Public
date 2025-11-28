package org.firstinspires.ftc.teamcode.subsystems.vision;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.limelightvision.LLResultTypes.FiducialResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.List;
import java.util.Locale;

public class AutoApriltag extends SubsystemBase {
    public Limelight3A limelight;
    public AutoApriltag(final HardwareMap hardwareMap) {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(50);
        limelight.start();
    }

    public List<FiducialResult> getTagResult() {
        return limelight.getLatestResult().getFiducialResults();
    }

    public String getRobotPosition(){
        List<FiducialResult> fiducialResult = limelight.getLatestResult().getFiducialResults();
        if (!fiducialResult.isEmpty()) {
            Pose3D robotPose = fiducialResult.get(0).getCameraPoseTargetSpace();
            return String.format(Locale.CHINA,"X: %.2f, Y: %.2f, Z: %.2f, Pitch: %.2f," +
                            " Roll: %.2f, Yaw: %.2f", robotPose.getPosition().x,
                    robotPose.getPosition().y, robotPose.getPosition().z,
                    robotPose.getOrientation().getPitch(), robotPose.getOrientation().getRoll(),
                    robotPose.getOrientation().getYaw());
        }
        return "No AprilTag Detected";
    }
}
