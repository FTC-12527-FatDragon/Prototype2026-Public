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
import org.firstinspires.ftc.teamcode.commands.autocommands.AutoBrakeCommand;
import org.firstinspires.ftc.teamcode.commands.autocommands.AutoDriveCommand;
import org.firstinspires.ftc.teamcode.subsystems.shooter.Shooter;

@Config
@Autonomous(name = "BlueFar 3+9", group = "Autos")
public class BlueFar extends AutoCommandBase {

    public PathChain Path1, Path2, Path3, Path4, Path5, Path6;

    @Override
    public boolean highSpeed() {
        return false;
    }

    public Command shootCommand() {
        return new SequentialCommandGroup(
                new InstantCommand(() -> shooter.setShooterState(Shooter.ShooterState.FAST)),
                new ParallelRaceGroup(
                        new TransitCommand(transit, intake, shooter, cds),
                        new WaitCommand(1750)
                ),
                new InstantCommand(() -> shooter.setShooterState(Shooter.ShooterState.STOP))
        );
    }

    public Command cycleCommand() {
        return new SequentialCommandGroup(
                new AutoDriveCommand(follower, Path3),
                new ParallelRaceGroup(
                        new AutoDriveCommand(follower, Path4, 1000, Path5),
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
                new AutoBrakeCommand(follower, Path6.endPose()),
                shootCommand()
        );
    }

    @Override
    public Pose getStartPose() {
        return new Pose(
                57.004,
                9.39,
                Math.toRadians(-90)
        );
    }

    @Override
    public Command runAutoCommand() {
        Path1 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(57.004, 9.390), new Pose(57.004, 12.549))
                )
                .setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(-90))
                .build();

        Path2 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(57.004, 12.549), new Pose(58.068, 14.464))
                )
                .setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(-67))
                .build();

        Path3 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(58.068, 14.464), new Pose(32.331, 14.251))
                )
                .setLinearHeadingInterpolation(Math.toRadians(-67), Math.toRadians(180))
                .build();

        Path4 = follower
                .pathBuilder()
                .addPath(new BezierLine(new Pose(32.331, 14.251), new Pose(8.295, 9.359)))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();

        Path5 = follower
                .pathBuilder()
                .addPath(new BezierLine(new Pose(8.295, 9.359), new Pose(32.331, 14.251)))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();

        Path6 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(32.331, 14.251), new Pose(58.068, 14.464))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(-67))
                .build();

        return new SequentialCommandGroup(
                new AutoDriveCommand(follower, Path1),
                new AutoDriveCommand(follower, Path2),
                new AutoBrakeCommand(follower, Path2.endPose()),
                shootCommand(),
                cycleCommand(),
                cycleCommand(),
                new AutoDriveCommand(follower, Path3)
        );
    }
}
