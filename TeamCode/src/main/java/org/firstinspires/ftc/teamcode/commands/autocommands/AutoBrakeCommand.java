package org.firstinspires.ftc.teamcode.commands.autocommands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.util.ElapsedTime;

public class AutoBrakeCommand extends CommandBase {
    private final Follower follower;
    private Pose holdPose;
    private PathChain holdPath;
    private final ElapsedTime timer;

    public AutoBrakeCommand(Follower follower, Pose holdPose) {
        this.follower = follower;
        this.holdPose = holdPose;
        timer = new ElapsedTime();
    }

    @Override
    public void initialize() {
        timer.reset();
        holdPath = follower.pathBuilder()
                .addPath(
                        new BezierLine(follower.getPose(), holdPose)
                )
                .setConstantHeadingInterpolation(holdPose.getHeading())
                .build();

        follower.followPath(holdPath);
    }

    @Override
    public void execute() {
        follower.update();
        follower.breakFollowing();
        holdPath = follower.pathBuilder()
                .addPath(
                        new BezierLine(follower.getPose(), holdPose)
                )
                .setConstantHeadingInterpolation(holdPose.getHeading())
                .build();
        follower.followPath(holdPath);
    }

    @Override
    public void end(boolean interrupted) {
        follower.breakFollowing();
    }

    @Override
    public boolean isFinished() {
        return timer.milliseconds() >= 250;
    }
}
