package org.firstinspires.ftc.teamcode.subsystems.vision;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.limelightvision.LLResultTypes.FiducialResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.ArrayList;
import java.util.List;

public class AutoApriltag extends SubsystemBase {
    public Limelight3A limelight;
    public AutoApriltag(final HardwareMap hardwareMap) {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(50);
        limelight.start();
    }
    public List getNumbers() {
        List<FiducialResult> fiducials = limelight.getLatestResult().getFiducialResults();
        List apriltagIDs = new ArrayList<>();
        for (FiducialResult fiducial : fiducials) {
            apriltagIDs.add(fiducial.getFiducialId()); // The ID number of the fiducial
        }
        return apriltagIDs;
    }
}
