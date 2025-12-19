package org.firstinspires.ftc.teamcode.opmodes.autos;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelRaceGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.commands.TransitCommand;
import org.firstinspires.ftc.teamcode.commands.autocommands.AutoBrakeCommand;
import org.firstinspires.ftc.teamcode.commands.autocommands.AutoDriveCommand;
import org.firstinspires.ftc.teamcode.subsystems.shooter.Shooter;

public class BlueNearInf extends AutoCommandBase {
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
                        new WaitCommand(2000)
                ),
                new InstantCommand(() -> shooter.setShooterState(Shooter.ShooterState.STOP))
        );
    }

    public Command cycleCommand() {
        return new SequentialCommandGroup(
                new ParallelRaceGroup(
                        new AutoDriveCommand(follower, Path5),
                        new IntakeCommand(transit, intake, cds)
                ),
                new ParallelRaceGroup(
                        new AutoDriveCommand(follower, Path6),
                        new IntakeCommand(transit, intake, cds)
                ),
                new ParallelRaceGroup(
                        new AutoDriveCommand(follower, Path7),
                        new IntakeCommand(transit, intake, cds)
                ),
                new AutoBrakeCommand(follower, Path7.endPose()),
                shootCommand()
        );
    }

    @Override
    public Pose getStartPose() {
        return new Pose(27.226, 131.025, Math.toRadians(-36));
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
                        new BezierLine(new Pose(27.226, 131.025), new Pose(51.049, 91.675))
                )
                .setLinearHeadingInterpolation(Math.toRadians(-36), Math.toRadians(-50))
                .build();

        Path2 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(51.049, 91.675), new Pose(47.007, 62.747))
                )
                .setLinearHeadingInterpolation(Math.toRadians(-50), Math.toRadians(180))
                .build();

        Path3 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(47.007, 62.747), new Pose(17.442, 58.068))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();

        Path4 = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(17.442, 58.068),
                                new Pose(52.963, 64.024),
                                new Pose(51.049, 91.675)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(-50))
                .build();

        Path5 = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(51.049, 91.675),
                                new Pose(50.836, 57.430),
                                new Pose(16.378, 62.322)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-50), Math.toRadians(180))
                .build();

        Path6 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(16.378, 62.322), new Pose(13.188, 59.344))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(160))
                .build();

        Path7 = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(13.188, 59.344),
                                new Pose(52.750, 59.982),
                                new Pose(51.049, 91.888)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(160), Math.toRadians(-50))
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
                new AutoBrakeCommand(follower, Path4.endPose()),
                cycleCommand()
        );
    }
}
