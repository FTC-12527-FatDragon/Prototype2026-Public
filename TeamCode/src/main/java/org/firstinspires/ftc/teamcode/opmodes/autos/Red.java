package org.firstinspires.ftc.teamcode.opmodes.autos;

import static org.firstinspires.ftc.teamcode.tests.Tuning.follower;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.subsystems.drive.Constants;

@Config
@Autonomous(name = "Red", group = "Autos")
public class Red extends AutoCommandBase {
    Follower follower;


    public PathChain Path1;
    public PathChain Path2;
    public PathChain Path3;
    public PathChain Path4;
    public PathChain Path5;
    public PathChain Path6;
    public PathChain Path7;
    public PathChain Path8;
    public PathChain Path9;
    public PathChain Path10;

    @Override
    public Command runAutoCommand() {
        follower = Constants.createFollower(hardwareMap);

        Path1 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(104.864, 134.467), new Pose(84.627, 83.958))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(225))
                .build();

        Path2 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(84.627, 83.958), new Pose(102.188, 83.791))
                )
                .setLinearHeadingInterpolation(Math.toRadians(225), Math.toRadians(0))
                .build();

        Path3 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(102.188, 83.791), new Pose(120.084, 83.958))
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        Path4 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(120.084, 83.958), new Pose(84.794, 83.958))
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(225))
                .build();

        Path5 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(84.794, 83.958), new Pose(103.024, 60.042))
                )
                .setLinearHeadingInterpolation(Math.toRadians(225), Math.toRadians(0))
                .build();

        Path6 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(103.024, 60.042), new Pose(119.749, 60.042))
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        Path7 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(119.749, 60.042), new Pose(84.627, 83.958))
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(225))
                .build();

        Path8 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(84.627, 83.958), new Pose(103.693, 35.624))
                )
                .setLinearHeadingInterpolation(Math.toRadians(225), Math.toRadians(0))
                .build();

        Path9 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(103.693, 35.624), new Pose(120.418, 35.624))
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        Path10 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(120.418, 35.624), new Pose(78.272, 16.557))
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(240))
                .build();

        return new SequentialCommandGroup();
    }
}
