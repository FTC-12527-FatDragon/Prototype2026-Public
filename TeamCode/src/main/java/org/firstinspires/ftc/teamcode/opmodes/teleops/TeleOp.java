package org.firstinspires.ftc.teamcode.opmodes.teleops;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.bylazar.configurables.annotations.Configurable;

import org.firstinspires.ftc.teamcode.commands.HeadResetCommand;
import org.firstinspires.ftc.teamcode.commands.TeleopDriveCommand;
import org.firstinspires.ftc.teamcode.subsystems.drive.Drive;
import org.firstinspires.ftc.teamcode.utils.FunctionalButton;

@Configurable
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOp")
public class TeleOp extends CommandOpMode {
    private Drive drive;

    private GamepadEx gamepadEx1;

    @Override
    public void initialize() {
        drive = new Drive(hardwareMap);
        gamepadEx1 = new GamepadEx(gamepad1);

        drive.setDefaultCommand(new TeleopDriveCommand(drive, gamepadEx1));

        new FunctionalButton(
                () -> gamepadEx1.getButton(GamepadKeys.Button.LEFT_STICK_BUTTON))
                .whenPressed(
                        () -> new HeadResetCommand(drive)
                );
    }

    @Override
    public void run() {
        CommandScheduler.getInstance().run();
    }
}
