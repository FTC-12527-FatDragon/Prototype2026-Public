package org.firstinspires.ftc.teamcode.opmodes.teleops;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.commands.TeleOpDriveCommand;
import org.firstinspires.ftc.teamcode.commands.TransitCommand;
import org.firstinspires.ftc.teamcode.commands.TransitCommandWithShooter;
import org.firstinspires.ftc.teamcode.subsystems.drive.Constants;
import org.firstinspires.ftc.teamcode.subsystems.drive.MecanumDriveOTOS;
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake;
import org.firstinspires.ftc.teamcode.subsystems.shooter.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.shooter.ShooterConstants;
import org.firstinspires.ftc.teamcode.subsystems.transit.Transit;
import org.firstinspires.ftc.teamcode.utils.FunctionalButton;

@Config
@Configurable
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOpCY")
public class TeleOpSolo extends CommandOpMode {
    private MecanumDriveOTOS drive;

    private Follower follower;

    private GamepadEx gamepadEx1;

    private Shooter shooter;

    private Transit transit;

    private Intake intake;

    private Telemetry telemetryM;

    private boolean[] isAuto = {false};

    @Override
    public void initialize() {
        drive = new MecanumDriveOTOS(hardwareMap);
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
                () -> gamepadEx1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) >= 0.5
        ).whenHeld(
                new InstantCommand(() -> transit.setLimitServoState(Transit.LimitServoState.CLOSE))
                        .andThen(new WaitCommand(300))
                        .alongWith(new IntakeCommand(transit, intake))
        );

        new FunctionalButton(
                () -> gamepadEx1.getButton(GamepadKeys.Button.RIGHT_BUMPER)
        ).whenPressed(
                new InstantCommand(() -> shooter.toggleShooterState(Shooter.ShooterState.FAST))
        );

        new FunctionalButton(
                () -> gamepadEx1.getButton(GamepadKeys.Button.LEFT_BUMPER)
        ).whenPressed(
                new InstantCommand(() -> shooter.toggleShooterState(Shooter.ShooterState.SLOW))
        );

        new FunctionalButton(
                () -> gamepadEx1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) >= 0.5
        ).whenHeld(
                new InstantCommand(() -> transit.setLimitServoState(Transit.LimitServoState.OPEN))
                        .andThen(new WaitCommand(300))
                        .andThen(new TransitCommandWithShooter(transit, shooter))
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
        TelemetryPacket packet = new TelemetryPacket();
        packet.put("ShooterVelocity", shooter.getAverageVelocity());
        packet.put("StopTime", transit.stopTime);
        packet.put("AtFast", shooter.shooterState == Shooter.ShooterState.FAST
                && shooter.getAverageVelocity() > ShooterConstants.fastVelocity);
        packet.put("AtSlow", shooter.shooterState == Shooter.ShooterState.SLOW
                && shooter.getAverageVelocity() > ShooterConstants.slowVelocity);
        FtcDashboard.getInstance().sendTelemetryPacket(packet);
    }
}
