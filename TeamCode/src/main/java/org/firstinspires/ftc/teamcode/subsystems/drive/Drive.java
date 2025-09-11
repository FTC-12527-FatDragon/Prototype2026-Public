package org.firstinspires.ftc.teamcode.subsystems.drive;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Drive extends SubsystemBase {
    private static Follower follower;

    public Drive(HardwareMap hardwareMap) {
        follower = Constants.createFollower(hardwareMap);
        follower.startTeleopDrive();
        follower.setPose(new Pose(0, 0, 0));
    }

    public void setTeleOpDrive(double x, double y, double heading) {
        follower.setTeleOpDrive(x, y, heading, false);
    }

    public Pose getPose() {
        return follower.getPose();
    }
}
