package org.firstinspires.ftc.teamcode.opmodes.autos;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelRaceGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


import org.firstinspires.ftc.teamcode.commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.commands.TransitCommand;
import org.firstinspires.ftc.teamcode.commands.autocommands.AutoBrakeCommand;
import org.firstinspires.ftc.teamcode.commands.autocommands.AutoDriveCommand;
import org.firstinspires.ftc.teamcode.subsystems.shooter.Shooter;

@Config
@Autonomous(name = "RedNear 3+9", group = "Autos")
public class RedNear extends AutoCommandBase {
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
    public PathChain Path11;
    public PathChain Path12;
    public PathChain Path13;

    @Override
    public Pose getStartPose() {
        return new Pose(116.561, 131.238, Math.toRadians(-144));
    }

    @Override
    public boolean highSpeed() {
        return false;
    }

    @Override
    public Command runAutoCommand() {
        Path1 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(116.561, 131.238), new Pose(88.484, 87.846))
                )
                .setLinearHeadingInterpolation(Math.toRadians(-144), Math.toRadians(-130))
                .build();

        Path2 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(88.484, 87.846), new Pose(95.929, 84.018))
                )
                .setLinearHeadingInterpolation(Math.toRadians(-130), Math.toRadians(0))
                .build();

        Path3 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(95.929, 84.018), new Pose(119.965, 83.805))
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        Path4 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(119.965, 83.805), new Pose(111.456, 82.316))
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        Path5 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(111.456, 82.316), new Pose(121.453, 79.976))
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        Path6 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(121.453, 79.976), new Pose(93.377, 92.313))
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-130))
                .build();

        Path7 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(93.377, 92.313), new Pose(100.396, 60.408))
                )
                .setLinearHeadingInterpolation(Math.toRadians(-130), Math.toRadians(0))
                .build();

        Path8 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(100.396, 60.408), new Pose(124.006, 60.620))
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        Path9 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(124.006, 60.620), new Pose(93.164, 92.100))
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-130))
                .build();

        Path10 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(93.164, 92.100), new Pose(100.609, 35.947))
                )
                .setLinearHeadingInterpolation(Math.toRadians(-130), Math.toRadians(0))
                .build();

        Path11 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(100.609, 35.947), new Pose(123.581, 36.372))
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        Path12 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(123.581, 36.372), new Pose(93.164, 92.313))
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-130))
                .build();

        Path13 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(93.164, 92.313), new Pose(104.650, 72.744))
                )
                .setLinearHeadingInterpolation(Math.toRadians(-130), Math.toRadians(-130))
                .build();

        return new SequentialCommandGroup(
                new AutoDriveCommand(follower, Path1),
                new AutoBrakeCommand(follower, Path1.endPose(), 400),
                new InstantCommand(() -> shooter.setShooterState(Shooter.ShooterState.SLOW)),
                new ParallelRaceGroup(
                        new TransitCommand(transit, intake, shooter, cds),
                        new WaitCommand(2000)
                ),
                new InstantCommand(() -> shooter.setShooterState(Shooter.ShooterState.STOP)),
                new AutoDriveCommand(follower, Path2),
                new ParallelRaceGroup(
                        new AutoDriveCommand(follower, Path3),
                        new IntakeCommand(transit, intake, cds)
                ),
                new AutoBrakeCommand(follower, Path3.endPose()),
                new ParallelRaceGroup(
                        new AutoDriveCommand(follower, Path4),
                        new IntakeCommand(transit, intake, cds)
                ),
                new AutoBrakeCommand(follower, Path4.endPose()),
                new ParallelRaceGroup(
                        new AutoDriveCommand(follower, Path5),
                        new IntakeCommand(transit, intake, cds)
                ),
                new ParallelRaceGroup(
                        new AutoDriveCommand(follower, Path6),
                        new IntakeCommand(transit, intake, cds)
                ),
                new AutoBrakeCommand(follower, Path6.endPose()),
                new InstantCommand(() -> shooter.setShooterState(Shooter.ShooterState.SLOW)),
                new ParallelRaceGroup(
                        new TransitCommand(transit, intake, shooter, cds),
                        new WaitCommand(2000)
                ),
                new InstantCommand(() -> shooter.setShooterState(Shooter.ShooterState.STOP)),
                new AutoDriveCommand(follower, Path7),
                new ParallelRaceGroup(
                        new AutoDriveCommand(follower, Path8),
                        new IntakeCommand(transit, intake, cds)
                ),
                new ParallelRaceGroup(
                        new AutoDriveCommand(follower, Path9),
                        new IntakeCommand(transit, intake, cds)
                ),
                new AutoBrakeCommand(follower, Path9.endPose()),
                new InstantCommand(() -> shooter.setShooterState(Shooter.ShooterState.SLOW)),
                new ParallelRaceGroup(
                        new TransitCommand(transit, intake, shooter, cds),
                        new WaitCommand(2000)
                ),
                new InstantCommand(() -> shooter.setShooterState(Shooter.ShooterState.STOP)),
                new AutoDriveCommand(follower, Path10),
                new ParallelRaceGroup(
                        new AutoDriveCommand(follower, Path11),
                        new IntakeCommand(transit, intake, cds)
                ),
                new ParallelRaceGroup(
                        new AutoDriveCommand(follower, Path12),
                        new IntakeCommand(transit, intake, cds)
                ),
                new AutoBrakeCommand(follower, Path12.endPose()),
                new InstantCommand(() -> shooter.setShooterState(Shooter.ShooterState.SLOW)),
                new ParallelRaceGroup(
                        new TransitCommand(transit, intake, shooter, cds),
                        new WaitCommand(2000)
                ),
                new InstantCommand(() -> shooter.setShooterState(Shooter.ShooterState.STOP)),
                new AutoDriveCommand(follower, Path13)
        );
    }
}