package org.firstinspires.ftc.teamcode.opmodes.autos;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.commands.autoCommands.AutoDriveCommand;
import org.firstinspires.ftc.teamcode.commands.TransitCommand;
import org.firstinspires.ftc.teamcode.subsystems.cds.CDS;
import org.firstinspires.ftc.teamcode.subsystems.drive.Constants;
import org.firstinspires.ftc.teamcode.subsystems.drive.MecanumDriveOTOS;
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake;
import org.firstinspires.ftc.teamcode.subsystems.shooter.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.transit.Transit;

@Config
@Autonomous(name = "Red", group = "Autos")
public class Red extends AutoCommandBase {
    private MecanumDriveOTOS drive;
    private Shooter shooter;
    private Transit transit;
    private Intake intake;
    private CDS cds;
    private Telemetry telemetryM;
    private Follower follower;


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
        return new SequentialCommandGroup();
    }

    @Override
    public Command runAutoCommand() {
        follower = Constants.createFollower(hardwareMap);

        path1 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(104.864, 134.467), new Pose(84.627, 83.958))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(225))
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
                new InstantCommand(() -> shooter.setShooterState(Shooter.ShooterState.SLOW)),
                new AutoDriveCommand(follower, path1),
                //new AutoShootCommand(transit, intake, shooter),
                new ParallelCommandGroup(
                        new TransitCommand(transit, intake, shooter),
                        new WaitUntilCommand(() -> shooter.getBalls() < 1)
                ),
                new InstantCommand(() -> shooter.setShooterState(Shooter.ShooterState.STOP)),
                new AutoDriveCommand(follower, path2),
                new ParallelCommandGroup(
                        new AutoDriveCommand(follower, path3),
                        new IntakeCommand(transit, intake)
                ),
                new InstantCommand(() -> shooter.setShooterState(Shooter.ShooterState.SLOW)),
                new AutoDriveCommand(follower, path4),
                new ParallelCommandGroup(
                        new TransitCommand(transit, intake, shooter),
                        new WaitUntilCommand(() -> shooter.getBalls() < 1)
                ),
                new InstantCommand(() -> shooter.setShooterState(Shooter.ShooterState.STOP)),
                new AutoDriveCommand(follower, path5),
                new ParallelCommandGroup(
                        new AutoDriveCommand(follower, path6),
                        new IntakeCommand(transit, intake)
                ),
                new InstantCommand(() -> shooter.setShooterState(Shooter.ShooterState.SLOW)),
                new AutoDriveCommand(follower, path7),
                new ParallelCommandGroup(
                        new TransitCommand(transit, intake, shooter),
                        new WaitUntilCommand(() -> shooter.getBalls() < 1)
                ),
                new InstantCommand(() -> shooter.setShooterState(Shooter.ShooterState.STOP)),
                new AutoDriveCommand(follower, path8),
                new ParallelCommandGroup(
                        new AutoDriveCommand(follower, path9),
                        new IntakeCommand(transit, intake)
                ),
                new InstantCommand(() -> shooter.setShooterState(Shooter.ShooterState.FAST)),
                new AutoDriveCommand(follower, path10),
                new ParallelCommandGroup(
                        new TransitCommand(transit, intake, shooter),
                        new WaitUntilCommand(() -> shooter.getBalls() < 1)
                )
        );
    }
}
