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
import org.firstinspires.ftc.teamcode.commands.autocommands.AutoBrakeCommand;
import org.firstinspires.ftc.teamcode.commands.autocommands.AutoDriveCommand;
import org.firstinspires.ftc.teamcode.subsystems.shooter.Shooter;

@Config
@Autonomous(name = "BlueFar 3+9", group = "Autos")
public class RedFar extends AutoCommandBase {
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
        return new Pose(57.004, 8.721, Math.toRadians(-90));
    }

    @Override
    public boolean highSpeed() {
        return true;
    }

    private Command cycleTry1() {
        return new SequentialCommandGroup(
                new ParallelRaceGroup(
                        new AutoDriveCommand(follower, Path6),
                        new IntakeCommand(transit, intake, cds)
                ),
                new ParallelRaceGroup(
                        new AutoDriveCommand(follower, Path7),
                        new IntakeCommand(transit, intake, cds)
                ),
                new ParallelRaceGroup(
                        new AutoDriveCommand(follower, Path8, 1200),
                        new IntakeCommand(transit, intake, cds)
                ),
                new ParallelRaceGroup(
                        new AutoDriveCommand(follower, Path12),
                        new IntakeCommand(transit, intake, cds)
                ),
                new AutoBrakeCommand(follower, Path11.endPose()),
                new InstantCommand(() -> shooter.setShooterState(Shooter.ShooterState.FAST)),
                new ParallelRaceGroup(
                        new TransitCommand(transit, intake, shooter, cds),
                        new WaitCommand(1750)
                ),
                new InstantCommand(() -> shooter.setShooterState(Shooter.ShooterState.STOP))
        );
    }

    private Command cycleTry2() {
        return new SequentialCommandGroup(
                new ParallelRaceGroup(
                        new AutoDriveCommand(follower, Path6),
                        new IntakeCommand(transit, intake, cds)
                ),
                new ParallelRaceGroup(
                        new AutoDriveCommand(follower, Path7),
                        new IntakeCommand(transit, intake, cds)
                ),
                new ParallelRaceGroup(
                        new AutoDriveCommand(follower, Path8, 1200),
                        new IntakeCommand(transit, intake, cds)
                ),
                new ParallelRaceGroup(
                        new AutoDriveCommand(follower, Path9),
                        new IntakeCommand(transit, intake, cds)
                ),
                new ParallelRaceGroup(
                        new AutoDriveCommand(follower, Path10, 500),
                        new IntakeCommand(transit, intake, cds)
                ),
                new ParallelRaceGroup(
                        new AutoDriveCommand(follower, Path11),
                        new IntakeCommand(transit, intake, cds)
                ),
                new AutoBrakeCommand(follower, Path11.endPose()),
                new InstantCommand(() -> shooter.setShooterState(Shooter.ShooterState.FAST)),
                new ParallelRaceGroup(
                        new TransitCommand(transit, intake, shooter, cds),
                        new WaitCommand(1750)
                ),
                new InstantCommand(() -> shooter.setShooterState(Shooter.ShooterState.STOP))
        );
    }

    @Override
    public Command runAutoCommand() {
        Path1 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(87.004, 8.721), new Pose(87.092, 13.188))
                )
                .setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(-90))
                .build();

        Path2 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(87.092, 13.188), new Pose(84.082, 17.442))
                )
                .setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(246))
                .build();

        Path3 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(84.082, 17.442), new Pose(97.220, 35.521))
                )
                .setLinearHeadingInterpolation(Math.toRadians(246), Math.toRadians(0))
                .build();

        Path4 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(97.220, 35.521), new Pose(124.000, 35.521))
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        Path5 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(124.000, 35.521), new Pose(84.000, 17.442))
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(246))
                .build();

        Path6 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(84.000, 17.442), new Pose(91.000, 17.442))
                )
                .setLinearHeadingInterpolation(Math.toRadians(246), Math.toRadians(0))
                .build();

        Path7 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(91.000, 17.442), new Pose(106.000, 14.889))
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        Path8 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(106.000, 14.889), new Pose(132.000, 9.572))
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        Path9 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(132.000, 9.572), new Pose(122.517, 14.038))
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        Path10 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(122.517, 14.038), new Pose(132.495, 15.102))
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        Path11 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(132.495, 15.102), new Pose(84.245, 17.241))
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(246))
                .build();

        // 8 end to 11 end
        Path12 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(Path8.endPose(), Path11.endPose())
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(-66))
                .build();

        return new SequentialCommandGroup(
                new ParallelCommandGroup(new AutoDriveCommand(follower, Path1),
                        new SequentialCommandGroup(new WaitCommand(150),
                                new InstantCommand(() -> shooter.setShooterState(Shooter.ShooterState.FAST)))),
                //new InstantCommand(() -> shooter.setShooterState(Shooter.ShooterState.FAST)),
                new AutoDriveCommand(follower, Path2),
                new AutoBrakeCommand(follower, Path2.endPose()),
                new ParallelRaceGroup(
                        new TransitCommand(transit, intake, shooter, cds),
                        new WaitCommand(1750)
                ),
                new InstantCommand(() -> shooter.setShooterState(Shooter.ShooterState.STOP)),
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
                new InstantCommand(() -> shooter.setShooterState(Shooter.ShooterState.FAST)),
                new ParallelRaceGroup(
                        new TransitCommand(transit, intake, shooter, cds),
                        new WaitCommand(1750)
                ),
                new InstantCommand(() -> shooter.setShooterState(Shooter.ShooterState.STOP)),
                cycleTry1(),
                cycleTry2(),
                cycleTry2()
        );
    }
}