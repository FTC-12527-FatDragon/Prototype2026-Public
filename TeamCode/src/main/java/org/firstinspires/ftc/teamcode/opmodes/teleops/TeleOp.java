package org.firstinspires.ftc.teamcode.opmodes.teleops;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.commands.IntakeToggleCommand;
import org.firstinspires.ftc.teamcode.commands.TeleOpDriveCommand;
import org.firstinspires.ftc.teamcode.commands.TeleOpPathCommand;
import org.firstinspires.ftc.teamcode.subsystems.drive.Constants;
import org.firstinspires.ftc.teamcode.subsystems.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake;
import org.firstinspires.ftc.teamcode.subsystems.shooter.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.transit.Transit;
import org.firstinspires.ftc.teamcode.utils.FunctionalButton;
import org.firstinspires.ftc.teamcode.utils.Pose2dToPose;
import org.firstinspires.ftc.teamcode.utils.Util;

@Configurable
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOpCY")
public class TeleOp extends CommandOpMode {
    private MecanumDrive drive;
    private Follower follower;

    private GamepadEx gamepadEx1;

    private Shooter shooter;

    private Transit transit;

    private Intake intake;

    private Telemetry telemetryM;

    private boolean[] isAuto = {false};

    @Override
    public void initialize() {
        drive = new MecanumDrive(hardwareMap);
        gamepadEx1 = new GamepadEx(gamepad1);
        follower = Constants.createFollower(hardwareMap);
        shooter = new Shooter(hardwareMap);
        transit = new Transit(hardwareMap);
        intake = new Intake(hardwareMap);

        drive.setDefaultCommand(new TeleOpDriveCommand(drive, gamepadEx1, isAuto));

        new FunctionalButton(
                () -> gamepadEx1.getButton(GamepadKeys.Button.LEFT_STICK_BUTTON)
        ).whenPressed(
                new InstantCommand(() -> drive.reset(0))
        );

//        new FunctionalButton(
//                () -> gamepadEx1.getButton(GamepadKeys.Button.A)
//        ).whenHeld(
//                new ShootCommand(shooter, ShooterSpeedCalc.calcSpeed(drive.getPose()))
//        );

        new FunctionalButton(
                () -> gamepadEx1.getButton(GamepadKeys.Button.B)
        ).whenPressed(
                new IntakeToggleCommand(intake)
        );

//        new FunctionalButton(
//                () -> gamepadEx1.getButton(GamepadKeys.Button.B)
//        ).whenPressed(
//                new InstantCommand(() -> isAuto[0] = true)
//                        .andThen(new TeleOpPathCommand(follower,
//                                TeleOpPaths.buildPath(follower,
//                                        Util.Pose2DToPose(drive.getPose()),
//                                        new Pose(2, 2, 1))))
//                        .andThen(new InstantCommand(() -> isAuto[0] = false))
//        );
    }

    @Override
    public void run() {
        telemetryM = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        CommandScheduler.getInstance().run();
        telemetry.addData("X", drive.getPose().getX(DistanceUnit.MM));
        telemetry.addData("Y",  drive.getPose().getY(DistanceUnit.MM));
        telemetry.addData("Heading", drive.getPose().getHeading(AngleUnit.RADIANS));
        telemetry.addData("YawOffset",drive.getYawOffset());
        telemetry.update();
    }
}
