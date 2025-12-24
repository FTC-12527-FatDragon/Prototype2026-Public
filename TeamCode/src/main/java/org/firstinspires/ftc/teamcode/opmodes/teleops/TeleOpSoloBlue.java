package org.firstinspires.ftc.teamcode.opmodes.teleops;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.commands.TeleOpDriveCommand;
import org.firstinspires.ftc.teamcode.commands.TransitCommand;
import org.firstinspires.ftc.teamcode.subsystems.cds.CDS;
import org.firstinspires.ftc.teamcode.subsystems.drive.VisionMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake;
import org.firstinspires.ftc.teamcode.subsystems.shooter.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.transit.Transit;
import org.firstinspires.ftc.teamcode.utils.FunctionalButton;

@Config
@Configurable
@TeleOp(name = "TeleOp Solo Blue")
public class TeleOpSoloBlue extends CommandOpMode {
    private VisionMecanumDrive drive;

    private GamepadEx gamepadEx1;

    private Shooter shooter;

    private Transit transit;

    private Intake intake;

    private CDS cds;

    @Override
    public void initialize() {
        cds = new CDS(hardwareMap);
        drive = new VisionMecanumDrive(hardwareMap, VisionMecanumDrive.DriveState.BLUE, cds);
        gamepadEx1 = new GamepadEx(gamepad1);
        shooter = new Shooter(hardwareMap, false);
        transit = new Transit(hardwareMap);
        intake = new Intake(hardwareMap, true);

        drive.setDefaultCommand(new TeleOpDriveCommand(drive, gamepadEx1,
                () -> gamepadEx1.getButton(GamepadKeys.Button.A)));


        new FunctionalButton(
                () -> gamepadEx1.getButton(GamepadKeys.Button.B)
        ).whenPressed(
                new InstantCommand(() -> drive.visionCalibrate())
        );

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
                new IntakeCommand(transit, intake, cds)
        );

        new FunctionalButton(
                () -> gamepadEx1.getButton(GamepadKeys.Button.RIGHT_BUMPER)
        ).whenHeld(
                new InstantCommand(() -> shooter.setShooterState(Shooter.ShooterState.FAST))
        ).whenReleased(
                new InstantCommand(() -> shooter.setShooterState(Shooter.ShooterState.STOP))
        );

        new FunctionalButton(
                () -> gamepadEx1.getButton(GamepadKeys.Button.LEFT_BUMPER)
        ).whenHeld(
                new InstantCommand(() -> shooter.setShooterState(Shooter.ShooterState.SLOW))
        ).whenReleased(
                new InstantCommand(() -> shooter.setShooterState(Shooter.ShooterState.STOP))
        );

        new FunctionalButton(
                () -> gamepadEx1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) >= 0.5
        ).whenHeld(
                new TransitCommand(transit, intake, shooter, cds, drive)
                        .alongWith(new InstantCommand(() -> drive.visionCalibrate()))
        );

        new FunctionalButton(
                () -> gamepadEx1.getButton(GamepadKeys.Button.DPAD_LEFT)
        ).whenPressed(
                new InstantCommand(() -> cds.kill())
        );

//        new FunctionalButton(
//                () -> gamepadEx1.getButton(GamepadKeys.Button.DPAD_RIGHT)
//        ).whenHeld(
//                new ChooseCommand(transit, intake)
//        );
//
//        new FunctionalButton(
//                () -> gamepadEx1.getButton(GamepadKeys.Button.DPAD_DOWN)
//        ).whenPressed(
//                new InstantCommand(() -> transit.setChooseServoState(Transit.ChooseServoState.OPEN))
//        );
//
//        new FunctionalButton(
//                () -> gamepadEx1.getButton(GamepadKeys.Button.DPAD_UP)
//        ).whenPressed(
//                new InstantCommand(() -> transit.setChooseServoState(Transit.ChooseServoState.CLOSE))
//        );

//        new FunctionalButton(
//                () -> gamepadEx1.getButton(GamepadKeys.Button.DPAD_LEFT)
//        ).whenPressed(
//                new AdjustCommand(transit, intake, cds)
//        );


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
        CommandScheduler.getInstance().run();
        telemetry.addData("VisionPose", drive.getVisionPose() != null);
        telemetry.addData("X", drive.getPose().getX(DistanceUnit.INCH));
        telemetry.addData("Y",  drive.getPose().getY(DistanceUnit.INCH));
        telemetry.addData("Heading", drive.getPose().getHeading(AngleUnit.RADIANS));
        telemetry.addData("YawOffset",drive.getYawOffset());
        telemetry.addData("ShooterVelocity", shooter.shooterState.toString());
        telemetry.addData("Ball Num", cds.getBallNum());
        telemetry.addData("Gamepad Lx: ", gamepadEx1.getLeftX());
        telemetry.addData("Gamepad Ly: ", gamepadEx1.getLeftY());
        telemetry.addData("Gamepad Rx: ", gamepadEx1.getRightX());
        telemetry.addData("LF Power: ", drive.leftBackMotor.getPower());
        telemetry.addData("RF Power: ", drive.rightFrontMotor.getPower());
        telemetry.addData("LB Power: ", drive.leftBackMotor.getPower());
        telemetry.addData("RB Motor: ", drive.rightBackMotor.getPower());
        telemetry.addData("LF Mode: ", drive.leftFrontMotor.getMode());
        telemetry.update();
        TelemetryPacket packet = new TelemetryPacket();
        packet.put("ShooterVelocity", shooter.getVelocity());
        packet.put("StopTime", transit.stopTime);
        FtcDashboard.getInstance().sendTelemetryPacket(packet);
    }
}
