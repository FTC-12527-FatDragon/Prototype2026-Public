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
import org.firstinspires.ftc.teamcode.commands.autocommands.AutoDriveCommand;
import org.firstinspires.ftc.teamcode.subsystems.shooter.Shooter;

@Config
@Autonomous(name = "BlueNear 3+9", group = "Autos")
public class BlueNear extends AutoCommandBase {
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

    @Override
    public Pose getStartPose() {
        return new Pose(39.137, 134.428, Math.toRadians(0));
    }

    @Override
    public Command runAutoCommand() {
        Path1 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(39.137, 134.428), new Pose(54.877, 87.421))
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-50))
                .build();

        Path2 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(54.877, 87.421), new Pose(44.880, 83.805))
                )
                .setLinearHeadingInterpolation(Math.toRadians(-50), Math.toRadians(180))
                .build();

        Path3 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(44.880, 83.805), new Pose(19.994, 83.805))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();

        Path4 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(19.994, 83.805), new Pose(28.077, 76.999))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();

        Path5 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(28.077, 76.999), new Pose(16.378, 76.573))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();

        Path6 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(16.378, 76.573), new Pose(54.665, 87.846))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(-50))
                .build();

        Path7 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(54.665, 87.846), new Pose(44.880, 59.557))
                )
                .setLinearHeadingInterpolation(Math.toRadians(-50), Math.toRadians(180))
                .build();

        Path8 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(44.880, 59.557), new Pose(20.207, 59.557))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();

        Path9 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(20.207, 59.557), new Pose(54.877, 87.634))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(-50))
                .build();

        Path10 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(54.877, 87.634), new Pose(44.030, 35.521))
                )
                .setLinearHeadingInterpolation(Math.toRadians(-50), Math.toRadians(180))
                .build();

        Path11 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(44.030, 35.521), new Pose(20.419, 35.521))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();

        Path12 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(20.419, 35.521), new Pose(54.665, 87.634))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(-50))
                .build();

        return new SequentialCommandGroup(
                new AutoDriveCommand(follower, Path1),
                new InstantCommand(() -> shooter.setShooterState(Shooter.ShooterState.SLOW)),
                new ParallelRaceGroup(
                        new TransitCommand(transit, intake, shooter, cds),
                        new WaitCommand(1500)
                ),
                new InstantCommand(() -> shooter.setShooterState(Shooter.ShooterState.STOP)),
                new AutoDriveCommand(follower, Path2),
                new ParallelRaceGroup(
                        new AutoDriveCommand(follower, Path3),
                        new IntakeCommand(transit, intake, cds)
                ),
                new ParallelRaceGroup(
                        new AutoDriveCommand(follower, Path4),
                        new IntakeCommand(transit, intake, cds)
                ),
                new ParallelRaceGroup(
                        new AutoDriveCommand(follower, Path5),
                        new IntakeCommand(transit, intake, cds)
                ),
                new ParallelRaceGroup(
                        new AutoDriveCommand(follower, Path6),
                        new IntakeCommand(transit, intake, cds)
                ),
                new InstantCommand(() -> shooter.setShooterState(Shooter.ShooterState.SLOW)),
                new ParallelRaceGroup(
                        new TransitCommand(transit, intake, shooter, cds),
                        new WaitCommand(1500)
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
                new InstantCommand(() -> shooter.setShooterState(Shooter.ShooterState.SLOW)),
                new ParallelRaceGroup(
                        new TransitCommand(transit, intake, shooter, cds),
                        new WaitCommand(1500)
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
                new InstantCommand(() -> shooter.setShooterState(Shooter.ShooterState.SLOW)),
                new ParallelRaceGroup(
                        new TransitCommand(transit, intake, shooter, cds),
                        new WaitCommand(1500)
                ),
                new InstantCommand(() -> shooter.setShooterState(Shooter.ShooterState.STOP))
        );
    }
}