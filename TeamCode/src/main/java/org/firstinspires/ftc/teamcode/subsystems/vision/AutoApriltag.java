package org.firstinspires.ftc.teamcode.subsystems.vision;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.limelightvision.LLResultTypes.FiducialResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.List;

public class AutoApriltag extends SubsystemBase {
    public Limelight3A limelight;
    public AutoApriltag(final HardwareMap hardwareMap) {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(50);
        limelight.start();
    }

    public String getRobotPosition(){
        List<FiducialResult> fiducialResult = limelight.getLatestResult().getFiducialResults();
        if (!fiducialResult.isEmpty()) {
            FiducialResult result = fiducialResult.get(0);
            Pose3D robotPose = VisionConstants.getCameraFieldPose(result);
            if (robotPose != null) return robotPose.toString();
            return null;
        }
        return "No AprilTag Detected";
    }
}
