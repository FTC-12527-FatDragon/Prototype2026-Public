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
@Autonomous(name = "RedNear 3+9", group = "Autos")
public class RedNear extends AutoCommandBase {

    public PathChain Path1, Path2, Path3, Path4, Path5, Path6;
    public PathChain Path7, Path8, Path9, Path10, Path11, Path12, Path13;

    public Command shootCommand() {
        return new SequentialCommandGroup(
                new InstantCommand(() -> shooter.setShooterState(Shooter.ShooterState.SLOW)),
                new ParallelRaceGroup(
                        new TransitCommand(transit, intake, shooter, cds),
                        new WaitCommand(1750)
                ),
                new InstantCommand(() -> shooter.setShooterState(Shooter.ShooterState.STOP))
        );
    }

    @Override
    public Pose getStartPose() {
        return new Pose(
                118.688,                 // 144 - 27.013
                131.451,
                Math.toRadians(-144)      // 180 - (-36)
        );
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
                        new BezierLine(new Pose(118.688, 131.451), new Pose(93.802, 93.377))
                )
                .setLinearHeadingInterpolation(Math.toRadians(-144), Math.toRadians(225))
                .build();

        Path2 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(93.802, 93.377), new Pose(92.526, 60.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(225), Math.toRadians(0))
                .build();

        Path3 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(92.526, 60.000), new Pose(124.600, 59.800))
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        Path4 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(124.600, 59.800), new Pose(126.984, 69.341))
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-90))
                .build();

        Path5 = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(126.984, 69.341),
                                new Pose(77.637, 60.620),
                                new Pose(91.675, 96.993)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(-132))
                .build();

        Path6 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(91.675, 96.993), new Pose(94.653, 83.592))
                )
                .setLinearHeadingInterpolation(Math.toRadians(-132), Math.toRadians(0))
                .build();

        Path7 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(94.653, 83.592), new Pose(122.304, 83.167))
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        Path8 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(122.304, 83.167), new Pose(91.888, 96.780))
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-133))
                .build();

        Path9 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(91.888, 96.780), new Pose(92.313, 63.811))
                )
                .setLinearHeadingInterpolation(Math.toRadians(-133), Math.toRadians(0))
                .build();

        Path10 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(92.313, 63.811), new Pose(93.377, 35.309))
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        Path11 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(93.377, 35.309), new Pose(125.920, 35.309))
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        Path12 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(125.920, 35.309), new Pose(91.462, 97.418))
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(225))
                .build();

        Path13 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(91.462, 97.418), new Pose(98.056, 68.490))
                )
                .setLinearHeadingInterpolation(Math.toRadians(225), Math.toRadians(0))
                .build();

        return new SequentialCommandGroup(
                new AutoDriveCommand(follower, Path1),
//                new AutoBrakeCommand(follower, Path1.endPose()),
                shootCommand(),
                new AutoDriveCommand(follower, Path2),
                new ParallelRaceGroup(
                        new AutoDriveCommand(follower, Path3),
                        new IntakeCommand(transit, intake, cds)
                ),
                new ParallelRaceGroup(
                        new AutoDriveCommand(follower, Path4, 1000, Path5),
                        new IntakeCommand(transit, intake, cds)
                ),
                new ParallelRaceGroup(
                        new AutoDriveCommand(follower, Path5),
                        new IntakeCommand(transit, intake, cds)
                ),
//                new AutoBrakeCommand(follower, Path5.endPose()),
                shootCommand(),
                new AutoDriveCommand(follower, Path6),
                new ParallelRaceGroup(
                        new AutoDriveCommand(follower, Path7),
                        new IntakeCommand(transit, intake, cds)
                ),
                new ParallelRaceGroup(
                        new AutoDriveCommand(follower, Path8),
                        new IntakeCommand(transit, intake, cds)
                ),
//                new AutoBrakeCommand(follower, Path8.endPose()),
                shootCommand(),
                new ParallelRaceGroup(
                        new AutoDriveCommand(follower, Path10),
                        new IntakeCommand(transit, intake, cds)
                ),
                new ParallelRaceGroup(
                        new AutoDriveCommand(follower, Path10),
                        new IntakeCommand(transit, intake, cds)
                ),
                new ParallelRaceGroup(
                        new AutoDriveCommand(follower, Path11),
                        new IntakeCommand(transit, intake, cds)
                ),
                new ParallelRaceGroup(
                        new AutoDriveCommand(follower, Path12),
                        new IntakeCommand(transit, intake, cds)
                ),
//                new AutoBrakeCommand(follower, Path11.endPose()),
                shootCommand(),
                new AutoDriveCommand(follower, Path13)
        );
    }
}
