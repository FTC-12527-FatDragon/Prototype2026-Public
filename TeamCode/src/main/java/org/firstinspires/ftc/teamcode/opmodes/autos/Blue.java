package org.firstinspires.ftc.teamcode.opmodes.autos;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
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
@Autonomous(name = "Blue", group = "Autos")
public class Blue extends AutoCommandBase {
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
        Path1 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(33.519, 135.700), new Pose(59.735, 83.891))
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-44))
                .build();

        Path2 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(59.735, 83.891), new Pose(41.196, 83.891))
                )
                .setLinearHeadingInterpolation(Math.toRadians(-44), Math.toRadians(180))
                .build();

        Path3 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(41.196, 83.891), new Pose(20.785, 83.704))
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        Path4 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(20.785, 83.704), new Pose(59.735, 83.891))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(-47))
                .build();

        Path5 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(59.735, 83.891), new Pose(41.384, 59.735))
                )
                .setLinearHeadingInterpolation(Math.toRadians(-47), Math.toRadians(180))
                .build();

        Path6 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(41.384, 59.735), new Pose(16.666, 59.547))
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        Path7 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(16.666, 59.547), new Pose(59.735, 83.891))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(-47))
                .build();

        Path8 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(59.735, 83.891), new Pose(41.758, 35.391))
                )
                .setLinearHeadingInterpolation(Math.toRadians(-47), Math.toRadians(180))
                .build();

        Path9 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(41.758, 35.391), new Pose(17.040, 35.766))
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        Path10 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(17.040, 35.766), new Pose(67.225, 17.228))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(-65))
                .build();

        return new SequentialCommandGroup(
                new InstantCommand(() -> shooter.setShooterState(Shooter.ShooterState.SLOW)),
                new AutoDriveCommand(follower, Path1),
                //new AutoShootCommand(transit, intake, shooter),
                new ParallelRaceGroup(
                        new TransitCommand(transit, intake, shooter),
                        new WaitCommand(2500)
                ),
                new InstantCommand(() -> shooter.setShooterState(Shooter.ShooterState.STOP)),
                new AutoDriveCommand(follower, Path2),
                new ParallelRaceGroup(
                        new AutoDriveCommand(follower, Path3),
                        new IntakeCommand(transit, intake)
//                        new InstantCommand(() -> shooter.setBalls(3))
                ),
                new InstantCommand(() -> shooter.setShooterState(Shooter.ShooterState.SLOW)),
                new AutoDriveCommand(follower, Path4),
                new ParallelRaceGroup(
                        new TransitCommand(transit, intake, shooter),
                        new WaitCommand(2500)
                ),
                new InstantCommand(() -> shooter.setShooterState(Shooter.ShooterState.STOP)),
                new AutoDriveCommand(follower, Path5),
                new ParallelRaceGroup(
                        new AutoDriveCommand(follower, Path6),
                        new IntakeCommand(transit, intake)
//                        new InstantCommand(() -> shooter.setBalls(3))
                ),
                new InstantCommand(() -> shooter.setShooterState(Shooter.ShooterState.SLOW)),
                new AutoDriveCommand(follower, Path7),
                new ParallelRaceGroup(
                        new TransitCommand(transit, intake, shooter),
                        new WaitCommand(2500)
                ),
                new InstantCommand(() -> shooter.setShooterState(Shooter.ShooterState.STOP)),
                new AutoDriveCommand(follower, Path8),
                new ParallelRaceGroup(
                        new AutoDriveCommand(follower, Path9),
                        new IntakeCommand(transit, intake)
//                        new InstantCommand(() -> shooter.setBalls(3))
                ),
                new InstantCommand(() -> shooter.setShooterState(Shooter.ShooterState.FAST)),
                new AutoDriveCommand(follower, Path10),
                new ParallelRaceGroup(
                        new TransitCommand(transit, intake, shooter),
                        new WaitCommand(2500)
                )
        );
    }
}