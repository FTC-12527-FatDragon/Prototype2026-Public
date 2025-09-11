package org.firstinspires.ftc.teamcode.opmodes.teleops;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.bylazar.configurables.annotations.Configurable;

import org.firstinspires.ftc.teamcode.commands.TeleopDriveCommand;
import org.firstinspires.ftc.teamcode.subsystems.drive.Drive;

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
    }

    @Override
    public void run() {
        CommandScheduler.getInstance().run();
    }
}
