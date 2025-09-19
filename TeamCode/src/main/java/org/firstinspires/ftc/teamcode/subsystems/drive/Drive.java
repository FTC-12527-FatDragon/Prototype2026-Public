package org.firstinspires.ftc.teamcode.subsystems.drive;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Drive extends SubsystemBase {
    private final Follower follower;

    public Drive(HardwareMap hardwareMap) {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(0, 0, 0));
        follower.startTeleopDrive();
    }

    public void setTeleOpDrive(double x, double y, double heading) {
        follower.setTeleOpDrive(x, y, heading, false);
    }

    public Pose getPose() {
        return follower.getPose();
    }

    public void resetHead() {
        follower.setPose(follower.getPose().setHeading(0));
    }

    @Override
    public void periodic() {
        follower.update();
    }
}
