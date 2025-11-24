package org.firstinspires.ftc.teamcode.opmodes.autos;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelRaceGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.commands.TransitCommand;
import org.firstinspires.ftc.teamcode.commands.autocommands.AutoDriveCommand;
import org.firstinspires.ftc.teamcode.subsystems.cds.CDS;
import org.firstinspires.ftc.teamcode.subsystems.drive.Constants;
import org.firstinspires.ftc.teamcode.subsystems.drive.MecanumDriveOTOS;
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake;
import org.firstinspires.ftc.teamcode.subsystems.shooter.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.transit.Transit;

@Config
@Autonomous(name = "Red", group = "Autos")
public class Red extends AutoCommandBase {
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
        return new Pose(104.864, 134.467, Math.toRadians(180));
    }

    @Override
    public Command runAutoCommand() {
        Path1 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(104.864, 134.467), new Pose(84.627, 83.958))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(230))
                .build();

        Path2 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(84.627, 83.958), new Pose(102.188, 83.791))
                )
                .setLinearHeadingInterpolation(Math.toRadians(230), Math.toRadians(0))
                .build();

        Path3 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(102.188, 83.791), new Pose(125, 83.958))
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        Path4 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(125, 83.958), new Pose(84.627, 82))
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(230))
                .build();

        Path5 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(84.527, 82), new Pose(103.024, 58))
                )
                .setLinearHeadingInterpolation(Math.toRadians(230), Math.toRadians(0))
                .build();

        Path6 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(103.024, 58), new Pose(135, 58))
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        Path7 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(135, 58), new Pose(84.627, 82))
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(230))
                .build();

        Path8 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(84.627, 82), new Pose(103.693, 37))
                )
                .setLinearHeadingInterpolation(Math.toRadians(230), Math.toRadians(0))
                .build();

        Path9 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(103.693, 37), new Pose(135, 37))
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        Path10 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(135, 37), new Pose(78.272, 16.557))
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(237))
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
                        new AutoDriveCommand(follower, Path3).andThen(new WaitCommand(500)),
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
                        new AutoDriveCommand(follower, Path6).andThen(new WaitCommand(500)),
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
                        new AutoDriveCommand(follower, Path9).andThen(new WaitCommand(500)),
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