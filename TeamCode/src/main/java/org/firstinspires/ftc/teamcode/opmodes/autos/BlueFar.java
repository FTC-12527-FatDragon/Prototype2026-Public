package org.firstinspires.ftc.teamcode.opmodes.autos;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelRaceGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


import org.firstinspires.ftc.teamcode.commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.commands.TransitCommand;
import org.firstinspires.ftc.teamcode.commands.autocommands.AutoDriveCommand;
import org.firstinspires.ftc.teamcode.subsystems.shooter.Shooter;

@Config
@Autonomous(name = "BlueFar 3+9", group = "Autos")
public class BlueFar extends AutoCommandBase {
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
    public Pose getStartPose() {
        return new Pose(63.869387755102046, 8.228571428571431, Math.toRadians(-90));
    }

    @Override
    public Command runAutoCommand() {
        Path1 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(63.869, 8.229), new Pose(71.902, 22.531))
                )
                .setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(-60))
                .build();

        Path2 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(71.902, 22.531), new Pose(41.731, 59.559))
                )
                .setLinearHeadingInterpolation(Math.toRadians(-60), Math.toRadians(180))
                .build();

        Path3 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(41.731, 59.559), new Pose(19.200, 59.559))
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        Path4 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(19.200, 59.559), new Pose(71.902, 22.531))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(-60))
                .build();

        Path5 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(71.902, 22.531), new Pose(41.731, 34.873))
                )
                .setLinearHeadingInterpolation(Math.toRadians(-60), Math.toRadians(180))
                .build();

        Path6 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(41.731, 34.873), new Pose(18.808, 35.461))
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        Path7 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(18.808, 35.461), new Pose(71.902, 22.727))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(-60))
                .build();

        Path8 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(71.902, 22.727), new Pose(84.441, 9.992))
                )
                .setLinearHeadingInterpolation(Math.toRadians(-60), Math.toRadians(0))
                .build();

        Path9 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(84.441, 9.992), new Pose(134.596, 10.188))
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        Path10 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(134.596, 10.188), new Pose(72.098, 22.531))
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-60))
                .build();

        return new SequentialCommandGroup(
                new AutoDriveCommand(follower, Path1),
                //new AutoShootCommand(transit, intake, shooter),
                new InstantCommand(() -> shooter.setShooterState(Shooter.ShooterState.SLOW)),
                new ParallelRaceGroup(
                        new TransitCommand(transit, intake, shooter, cds),
                        new WaitCommand(2500)
                ),
                new InstantCommand(() -> shooter.setShooterState(Shooter.ShooterState.STOP)),
                new AutoDriveCommand(follower, Path2),
                new ParallelRaceGroup(
                        new AutoDriveCommand(follower, Path3),
                        new IntakeCommand(transit, intake, cds)
//                        new InstantCommand(() -> shooter.setBalls(3))
                ),
                new ParallelRaceGroup(
                        new AutoDriveCommand(follower, Path4),
                        new IntakeCommand(transit, intake, cds)
                ),
                new InstantCommand(() -> shooter.setShooterState(Shooter.ShooterState.SLOW)),
                new ParallelRaceGroup(
                        new TransitCommand(transit, intake, shooter, cds),
                        new WaitCommand(2500)
                ),
                new InstantCommand(() -> shooter.setShooterState(Shooter.ShooterState.STOP)),
                new AutoDriveCommand(follower, Path5),
                new ParallelRaceGroup(
                        new AutoDriveCommand(follower, Path6),
                        new IntakeCommand(transit, intake, cds)
//                        new InstantCommand(() -> shooter.setBalls(3))
                ),
                new ParallelRaceGroup(
                        new AutoDriveCommand(follower, Path7),
                        new IntakeCommand(transit, intake, cds)
                ),
                new InstantCommand(() -> shooter.setShooterState(Shooter.ShooterState.SLOW)),
                new ParallelRaceGroup(
                        new TransitCommand(transit, intake, shooter, cds),
                        new WaitCommand(2500)
                ),
                new InstantCommand(() -> shooter.setShooterState(Shooter.ShooterState.STOP)),
                new AutoDriveCommand(follower, Path8),
                new ParallelRaceGroup(
                        new AutoDriveCommand(follower, Path9),
                        new IntakeCommand(transit, intake, cds)
//                        new InstantCommand(() -> shooter.setBalls(3))
                ),
                new ParallelRaceGroup(
                        new AutoDriveCommand(follower, Path10),
                        new IntakeCommand(transit, intake, cds)
                ),
                new InstantCommand(() -> shooter.setShooterState(Shooter.ShooterState.SLOW)),
                new ParallelRaceGroup(
                        new TransitCommand(transit, intake, shooter, cds),
                        new WaitCommand(2500)
                ),
                new InstantCommand(() -> shooter.setShooterState(Shooter.ShooterState.STOP))
        );
    }
}