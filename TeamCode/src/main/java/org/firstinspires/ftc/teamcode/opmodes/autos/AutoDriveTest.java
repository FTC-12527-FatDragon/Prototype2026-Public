package org.firstinspires.ftc.teamcode.opmodes.autos;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.commands.AutoDriveCommand;
import org.firstinspires.ftc.teamcode.subsystems.drive.Constants;

@Config
@Autonomous(name = "AutoDriveTest", group = "Autos")
public class AutoDriveTest extends AutoCommandBase {
    private Follower follower;

    public Pose startPose = new Pose(104.864, 134.467, Math.toRadians(180));
    public PathChain path1;
    public PathChain path2;
    public PathChain path3;
    public PathChain path4;
    public PathChain path5;
    public PathChain path6;
    public PathChain path7;
    public PathChain path8;
    public PathChain path9;
    public PathChain path10;

    private Command initializeCommand() {
        return new SequentialCommandGroup(
                new InstantCommand(() -> follower.setStartingPose(startPose))
        );
    }

    @Override
    public Command runAutoCommand() {
        follower = Constants.createFollower(hardwareMap);

        path1 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(startPose, new Pose(84.627, 83.958))
                )
                .setLinearHeadingInterpolation(startPose.getHeading(), Math.toRadians(225))
                .build();

        path2 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(84.627, 83.958), new Pose(102.188, 83.791))
                )
                .setLinearHeadingInterpolation(Math.toRadians(225), Math.toRadians(0))
                .build();

        path3 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(102.188, 83.791), new Pose(120.084, 83.958))
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        path4 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(120.084, 83.958), new Pose(84.794, 83.958))
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(225))
                .build();

        path5 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(84.794, 83.958), new Pose(103.024, 60.042))
                )
                .setLinearHeadingInterpolation(Math.toRadians(225), Math.toRadians(0))
                .build();

        path6 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(103.024, 60.042), new Pose(119.749, 60.042))
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        path7 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(119.749, 60.042), new Pose(84.627, 83.958))
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(225))
                .build();

        path8 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(84.627, 83.958), new Pose(103.693, 35.624))
                )
                .setLinearHeadingInterpolation(Math.toRadians(225), Math.toRadians(0))
                .build();

        path9 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(103.693, 35.624), new Pose(120.418, 35.624))
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        path10 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(120.418, 35.624), new Pose(78.272, 16.557))
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(240))
                .build();

        return new SequentialCommandGroup(
                initializeCommand(),
                new AutoDriveCommand(follower, path1)
        );
    }
}
