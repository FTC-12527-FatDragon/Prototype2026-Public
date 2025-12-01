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

    @Override
    public Pose getStartPose() {
        return new Pose(33.519, 135.700, Math.toRadians(0));
    }

    @Override
    public Command runAutoCommand() {
//        Path1 = follower
//                .pathBuilder()
//                .addPath(
//                        new BezierLine(new Pose(33.519, 135.700), new Pose(59.735, 83.891))
//                )
//                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-44))
//                .build();
//
//        Path2 = follower
//                .pathBuilder()
//                .addPath(
//                        new BezierLine(new Pose(59.735, 83.891), new Pose(41.196, 83.891))
//                )
//                .setLinearHeadingInterpolation(Math.toRadians(-44), Math.toRadians(180))
//                .build();
//
//        Path3 = follower
//                .pathBuilder()
//                .addPath(
//                        new BezierLine(new Pose(41.196, 83.891), new Pose(20.785, 83.704))
//                )
//                .setConstantHeadingInterpolation(Math.toRadians(180))
//                .build();
//
//        Path4 = follower
//                .pathBuilder()
//                .addPath(
//                        new BezierLine(new Pose(20.785, 83.704), new Pose(59.735, 83.891))
//                )
//                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(-47))
//                .build();
//
//        Path5 = follower
//                .pathBuilder()
//                .addPath(
//                        new BezierLine(new Pose(59.735, 83.891), new Pose(41.384, 59.735))
//                )
//                .setLinearHeadingInterpolation(Math.toRadians(-47), Math.toRadians(180))
//                .build();
//
//        Path6 = follower
//                .pathBuilder()
//                .addPath(
//                        new BezierLine(new Pose(41.384, 59.735), new Pose(16.666, 59.547))
//                )
//                .setConstantHeadingInterpolation(Math.toRadians(180))
//                .build();
//
//        Path7 = follower
//                .pathBuilder()
//                .addPath(
//                        new BezierLine(new Pose(16.666, 59.547), new Pose(59.735, 83.891))
//                )
//                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(-47))
//                .build();
//
//        Path8 = follower
//                .pathBuilder()
//                .addPath(
//                        new BezierLine(new Pose(59.735, 83.891), new Pose(41.758, 35.391))
//                )
//                .setLinearHeadingInterpolation(Math.toRadians(-47), Math.toRadians(180))
//                .build();
//
//        Path9 = follower
//                .pathBuilder()
//                .addPath(
//                        new BezierLine(new Pose(41.758, 35.391), new Pose(17.040, 35.766))
//                )
//                .setConstantHeadingInterpolation(Math.toRadians(180))
//                .build();
//
//        Path10 = follower
//                .pathBuilder()
//                .addPath(
//                        new BezierLine(new Pose(17.040, 35.766), new Pose(67.225, 17.228))
//                )
//                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(-65))
//                .build();

        Path1 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(37.616, 135.967), new Pose(59.167, 84.441))
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-50))
                .build();

        Path2 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(59.167, 84.441), new Pose(39.576, 86.008))
                )
                .setLinearHeadingInterpolation(Math.toRadians(-50), Math.toRadians(180))
                .build();

        Path3 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(39.576, 86.008), new Pose(21.747, 86.596))
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        Path4 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(21.747, 86.596), new Pose(58.971, 84.441))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(-50))
                .build();

        Path5 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(58.971, 84.441), new Pose(38.988, 63.478))
                )
                .setLinearHeadingInterpolation(Math.toRadians(-50), Math.toRadians(180))
                .build();

        Path6 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(38.988, 63.478), new Pose(22.139, 63.282))
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        Path7 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(22.139, 63.282), new Pose(59.167, 84.245))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(-50))
                .build();

        Path8 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(59.167, 84.245), new Pose(39.380, 40.751))
                )
                .setLinearHeadingInterpolation(Math.toRadians(-50), Math.toRadians(180))
                .build();

        Path9 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(39.380, 40.751), new Pose(22.139, 40.163))
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        Path10 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(22.139, 40.163), new Pose(59.167, 84.441))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(-50))
                .build();

        return new SequentialCommandGroup(
                new InstantCommand(() -> shooter.setShooterState(Shooter.ShooterState.SLOW)),
                new AutoDriveCommand(follower, Path1),
                //new AutoShootCommand(transit, intake, shooter),
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
                new InstantCommand(() -> shooter.setShooterState(Shooter.ShooterState.SLOW)),
                new AutoDriveCommand(follower, Path4),
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
                new InstantCommand(() -> shooter.setShooterState(Shooter.ShooterState.SLOW)),
                new AutoDriveCommand(follower, Path7),
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
                new InstantCommand(() -> shooter.setShooterState(Shooter.ShooterState.FAST)),
                new AutoDriveCommand(follower, Path10),
                new ParallelRaceGroup(
                        new TransitCommand(transit, intake, shooter, cds),
                        new WaitCommand(2500)
                ),
                new InstantCommand(() -> shooter.setShooterState(Shooter.ShooterState.STOP))
        );
    }
}