package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;

import org.firstinspires.ftc.teamcode.subsystems.drive.MecanumDrive;

public class TeleopDriveCommand extends CommandBase {
    private final MecanumDrive drive;
    private final GamepadEx gamepadEx;

    public TeleopDriveCommand(MecanumDrive drive, GamepadEx gamepadEx) {
        this.drive = drive;
        this.gamepadEx = gamepadEx;
        addRequirements(drive);
    }

    @Override
    public void execute() {
        drive.moveRobotFieldRelative(-gamepadEx.getLeftY(), -gamepadEx.getLeftX(), -gamepadEx.getRightX());
    }
}
