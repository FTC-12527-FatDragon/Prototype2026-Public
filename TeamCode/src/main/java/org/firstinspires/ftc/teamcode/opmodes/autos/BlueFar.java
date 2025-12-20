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

    public PathChain Path1, Path2, Path3, Path4, Path5, Path6, Path7, Path8, Path9;

    @Override
    public boolean highSpeed() {
        return true;
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
                new ParallelRaceGroup(
                        new AutoDriveCommand(follower, Path6, 2000, Path7),
                        new IntakeCommand(transit, intake, cds)
                ),
                new ParallelRaceGroup(
                        new AutoDriveCommand(follower, Path7, 2000, Path8),
                        new IntakeCommand(transit, intake, cds)
                ),
                new ParallelRaceGroup(
                        new AutoDriveCommand(follower, Path8, 2000, Path9),
                        new IntakeCommand(transit, intake, cds)
                ),
                new ParallelRaceGroup(
                        new AutoDriveCommand(follower, Path9),
                        new IntakeCommand(transit, intake, cds)
                ),
                new AutoBrakeCommand(follower, Path9.endPose()),
                shootCommand()
        );
    }

    @Override
    public Pose getStartPose() {
        return new Pose(
                144 - 86.996,
                8.721,
                Math.toRadians(180) - Math.toRadians(-90)
        );
    }

    @Override
    public Command runAutoCommand() {

        Path1 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(144 - 86.996, 8.721),
                        new Pose(144 - 86.996, 11.911)
                ))
                .setLinearHeadingInterpolation(
                        Math.toRadians(-90),
                        Math.toRadians(-90)
                )
                .build();

        Path2 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(144 - 86.996, 11.911),
                        new Pose(144 - 92.313, 13.613)
                ))
                .setLinearHeadingInterpolation(
                        Math.toRadians(-90),
                        Math.toRadians(180) - Math.toRadians(-108 - 2)
                )
                .build();

        Path3 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(144 - 92.313, 13.613),
                        new Pose(144 - 114.222, 14.251)
                ))
                .setLinearHeadingInterpolation(
                        Math.toRadians(180) - Math.toRadians(-108 - 2),
                        Math.toRadians(180) - Math.toRadians(0)
                )
                .build();

        Path4 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(144 - 114.222, 14.251),
                        new Pose(144 - 132.089, 11.911)
                ))
                .setLinearHeadingInterpolation(
                        Math.toRadians(180) - Math.toRadians(0),
                        Math.toRadians(180) - Math.toRadians(-21)
                )
                .build();

        Path5 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(144 - 132.089, 11.911),
                        new Pose(144 - 92.313, 13.613)
                ))
                .setLinearHeadingInterpolation(
                        Math.toRadians(180) - Math.toRadians(-21),
                        Math.toRadians(180) - Math.toRadians(-108)
                )
                .build();

        Path6 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(144 - 92.313, 13.613),
                        new Pose(144 - 131.876, 14.889)
                ))
                .setLinearHeadingInterpolation(
                        Math.toRadians(180) - Math.toRadians(-108),
                        Math.toRadians(180) - Math.toRadians(0)
                )
                .build();

        Path7 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(144 - 131.876, 14.889),
                        new Pose(144 - 125.069, 13.400)
                ))
                .setLinearHeadingInterpolation(
                        Math.toRadians(180) - Math.toRadians(0),
                        Math.toRadians(180) - Math.toRadians(0)
                )
                .build();

        Path8 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(144 - 125.069, 13.400),
                        new Pose(144 - 132.089, 11.911)
                ))
                .setLinearHeadingInterpolation(
                        Math.toRadians(180) - Math.toRadians(0),
                        Math.toRadians(180) - Math.toRadians(-21)
                )
                .build();

        Path9 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(144 - 132.089, 11.911),
                        new Pose(144 - 92.100, 13.613)
                ))
                .setLinearHeadingInterpolation(
                        Math.toRadians(180) - Math.toRadians(-21),
                        Math.toRadians(180) - Math.toRadians(-108)
                )
                .build();

        return new SequentialCommandGroup(
                new AutoDriveCommand(follower, Path1),
                new InstantCommand(() -> shooter.setShooterState(Shooter.ShooterState.FAST)),
                new AutoDriveCommand(follower, Path2),
                new AutoBrakeCommand(follower, Path2.endPose()),
                shootCommand(),
                new AutoDriveCommand(follower, Path3),
                new ParallelRaceGroup(
                        new AutoDriveCommand(follower, Path4),
                        new IntakeCommand(transit, intake, cds)
                ),
                new ParallelRaceGroup(
                        new AutoDriveCommand(follower, Path5),
                        new IntakeCommand(transit, intake, cds)
                ),
                new AutoBrakeCommand(follower, Path5.endPose()),
                shootCommand(),
                cycleCommand(),
                cycleCommand()
        );
    }
}
