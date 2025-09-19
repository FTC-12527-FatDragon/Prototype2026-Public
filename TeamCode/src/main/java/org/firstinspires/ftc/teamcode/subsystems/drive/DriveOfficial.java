package org.firstinspires.ftc.teamcode.subsystems.drive;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class DriveOfficial extends SubsystemBase {
    private final Follower follower;

    public DriveOfficial(HardwareMap hardwareMap, Pose startingPose) {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose == null ? new Pose() : startingPose);
        follower.update();
        follower.startTeleopDrive();
    }


}
