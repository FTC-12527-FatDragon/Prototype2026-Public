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
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.commands.TeleOpDriveCommand;
import org.firstinspires.ftc.teamcode.subsystems.cds.CDS;
import org.firstinspires.ftc.teamcode.subsystems.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.drive.MecanumDriveOTOS;
import org.firstinspires.ftc.teamcode.utils.FunctionalButton;

@Config
@Configurable
@TeleOp(name = "TeleOpDrive")
public class TeleOpDrive extends CommandOpMode {
    private MecanumDrive drive;

    private GamepadEx gamepadEx1;

    private CDS cds;

    @Override
    public void initialize() {
        cds = new CDS(hardwareMap);
        drive = new MecanumDrive(hardwareMap, MecanumDrive.DriveState.BLUE, cds);
        gamepadEx1 = new GamepadEx(gamepad1);

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
    }

    @Override
    public void run() {
        CommandScheduler.getInstance().run();
        telemetry.addData("X", drive.getPose().getX(DistanceUnit.INCH));
        telemetry.addData("Y",  drive.getPose().getY(DistanceUnit.INCH));
        telemetry.addData("Heading", drive.getPose().getHeading(AngleUnit.RADIANS));
        telemetry.addData("YawOffset",drive.getYawOffset());
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
        FtcDashboard.getInstance().sendTelemetryPacket(packet);
    }
}
