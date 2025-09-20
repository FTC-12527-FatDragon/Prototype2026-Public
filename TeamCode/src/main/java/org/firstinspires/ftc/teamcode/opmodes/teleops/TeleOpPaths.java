package org.firstinspires.ftc.teamcode.opmodes.teleops;

import static org.firstinspires.ftc.teamcode.tests.Tuning.follower;

import android.graphics.Point;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

public class TeleOpPaths {
    public static PathChain buildPath(Pose start, Pose end) {
        return follower.pathBuilder()
                .addPath(new BezierCurve(start, end))
                .setLinearHeadingInterpolation(start.getHeading(), end.getHeading())
                .build();
    }
}
