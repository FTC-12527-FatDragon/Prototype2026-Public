package org.firstinspires.ftc.teamcode.opmodes.teleops;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.commands.AdjustCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.commands.ShootCommand;
import org.firstinspires.ftc.teamcode.commands.TeleOpDriveCommand;
import org.firstinspires.ftc.teamcode.commands.TransitCommand;
import org.firstinspires.ftc.teamcode.commands.TransitCommandWithShooter;
import org.firstinspires.ftc.teamcode.subsystems.cds.CDS;
import org.firstinspires.ftc.teamcode.subsystems.drive.Constants;
import org.firstinspires.ftc.teamcode.subsystems.drive.MecanumDriveOTOS;
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake;
import org.firstinspires.ftc.teamcode.subsystems.shooter.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.shooter.ShooterConstants;
import org.firstinspires.ftc.teamcode.subsystems.transit.Transit;
import org.firstinspires.ftc.teamcode.utils.FunctionalButton;

import java.util.function.BooleanSupplier;

@Config
@Configurable
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOp1111")
public class TeleOp1111 extends CommandOpMode {
    private MecanumDriveOTOS drive;

    private Follower follower;

    private GamepadEx gamepadEx1;

    private Shooter shooter;

    private Transit transit;

    private Intake intake;

    private CDS cds;

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
        cds = new CDS(hardwareMap);

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
        ).whenHeld(
                new InstantCommand(() -> shooter.setOpenLoopPower(-1))
        ).whenReleased(
                new InstantCommand(() -> shooter.setOpenLoopPower(0))
        );

        new FunctionalButton(
                () -> gamepadEx1.getButton(GamepadKeys.Button.LEFT_BUMPER)
        ).whenHeld(
                new InstantCommand(() -> shooter.setOpenLoopPower(-0.88))
        ).whenReleased(
                new InstantCommand(() -> shooter.setOpenLoopPower(0))
        );

        new FunctionalButton(
                () -> gamepadEx1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) >= 0.5
        ).whenHeld(
                new TransitCommand(transit, intake)
        );

        new FunctionalButton(
                () -> gamepadEx1.getButton(GamepadKeys.Button.DPAD_DOWN)
        ).whenPressed(
                new InstantCommand(() -> transit.setChooseServoState(Transit.ChooseServoState.OPEN))
        );

        new FunctionalButton(
                () -> gamepadEx1.getButton(GamepadKeys.Button.DPAD_UP)
        ).whenPressed(
                new InstantCommand(() -> transit.setChooseServoState(Transit.ChooseServoState.CLOSE))
        );

        new FunctionalButton(
                () -> gamepadEx1.getButton(GamepadKeys.Button.DPAD_LEFT)
        ).whenPressed(
                new AdjustCommand(transit, intake, cds)
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
