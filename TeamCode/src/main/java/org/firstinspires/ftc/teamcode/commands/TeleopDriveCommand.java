package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;

import org.firstinspires.ftc.teamcode.subsystems.drive.Drive;

public class TeleopDriveCommand extends CommandBase {
    private final Drive drive;
    private GamepadEx gamepadEx;

    public TeleopDriveCommand(Drive drive, GamepadEx gamepadEx) {
        this.drive = drive;
        this.gamepadEx = gamepadEx;
        addRequirements(drive);
    }

    @Override
    public void execute() {
        drive.setTeleOpDrive(-gamepadEx.getLeftX(), gamepadEx.getLeftY(), gamepadEx.getRightX());
    }
}
