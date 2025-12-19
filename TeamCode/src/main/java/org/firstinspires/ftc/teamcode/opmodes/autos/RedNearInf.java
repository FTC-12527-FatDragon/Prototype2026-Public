package org.firstinspires.ftc.teamcode.opmodes.autos;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
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

@Autonomous(name = "Red Near INF", group = "Autos")
public class RedNearInf extends AutoCommandBase {
    public PathChain Path1;
    public PathChain Path2;
    public PathChain Path3;
    public PathChain Path4;
    public PathChain Path5;
    public PathChain Path6;
    public PathChain Path7;
    public Command shootCommand() {
        return new SequentialCommandGroup(
                new InstantCommand(() -> shooter.setShooterState(Shooter.ShooterState.SLOW)),
                new ParallelRaceGroup(
                        new TransitCommand(transit, intake, shooter, cds),
                        new WaitCommand(1500)
                ),
                new InstantCommand(() -> shooter.setShooterState(Shooter.ShooterState.STOP))
        );
    }

    public Command cycleCommand() {
        return new SequentialCommandGroup(
                new ParallelRaceGroup(
                        new AutoDriveCommand(follower, Path5, 1000),
                        new IntakeCommand(transit, intake, cds)
                ),
                new AutoBrakeCommand(follower, Path5.endPose()),
                new ParallelRaceGroup(
                        new AutoDriveCommand(follower, Path6, 500),
                        new IntakeCommand(transit, intake, cds)
                ),
                new AutoBrakeCommand(follower, Path6.endPose()),
                new WaitCommand(100),
                new ParallelRaceGroup(
                        new AutoDriveCommand(follower, Path7),
                        new IntakeCommand(transit, intake, cds)
                ),
                new ParallelCommandGroup(new AutoBrakeCommand(follower, Path7.endPose()),
                        shootCommand())
        );
    }

    @Override
    public Pose getStartPose() {
        return new Pose(116.774, 131.025, Math.toRadians(-144));
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
                        new BezierLine(new Pose(116.774, 131.025), new Pose(92.951, 91.675))
                )
                .setLinearHeadingInterpolation(Math.toRadians(-144), Math.toRadians(230))
                .build();

        Path2 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(92.951, 91.675), new Pose(96.993, 62.747))
                )
                .setLinearHeadingInterpolation(Math.toRadians(230), Math.toRadians(0))
                .build();

        Path3 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(96.993, 62.747), new Pose(126.558, 58.068))
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        Path4 = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(126.558, 58.068),
                                new Pose(91.037, 64.024),
                                new Pose(92.951, 91.675)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(230))
                .build();

        Path5 = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(92.951, 91.675),
                                new Pose(84.245, 64.261),
                                new Pose(122.449, 67.004)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(230), Math.toRadians(7))
                .build();

        Path6 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(122.449, 67.004), new Pose(130.090, 59.559))
                )
                .setLinearHeadingInterpolation(Math.toRadians(7), Math.toRadians(40))
                .build();

        Path7 = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(130.090, 59.559),
                                new Pose(91.250, 59.982),
                                new Pose(92.951, 91.888)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(40), Math.toRadians(230))
                .build();

        return new SequentialCommandGroup(
                new AutoDriveCommand(follower, Path1),
                new AutoBrakeCommand(follower, Path1.endPose()),
                shootCommand(),
                new AutoDriveCommand(follower, Path2),
                new AutoBrakeCommand(follower, Path2.endPose()),
                new ParallelRaceGroup(
                        new AutoDriveCommand(follower, Path3),
                        new IntakeCommand(transit, intake, cds)
                ),
                new ParallelRaceGroup(
                        new AutoDriveCommand(follower, Path4),
                        new IntakeCommand(transit, intake, cds)
                ),
                new ParallelCommandGroup(new AutoBrakeCommand(follower, Path4.endPose()),
                        shootCommand()),
                cycleCommand(),
                cycleCommand(),
                cycleCommand()
        );
    }
}
