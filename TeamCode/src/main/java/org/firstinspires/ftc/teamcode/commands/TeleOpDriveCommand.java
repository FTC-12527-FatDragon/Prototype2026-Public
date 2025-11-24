package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;

import org.firstinspires.ftc.teamcode.subsystems.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.drive.MecanumDriveOTOS;

public class TeleOpDriveCommand extends CommandBase {
    private final MecanumDriveOTOS drive;
    private final GamepadEx gamepadEx;
    private final boolean[] isAuto;

    public TeleOpDriveCommand(MecanumDriveOTOS drive, GamepadEx gamepadEx, boolean[] isAuto) {
        this.drive = drive;
        this.gamepadEx = gamepadEx;
        this.isAuto = isAuto;
        addRequirements(drive);
    }

    @Override
    public void execute() {
        if (!isAuto[0]) {
            if (Math.abs(gamepadEx.getLeftX()) > 0.03 || Math.abs(gamepadEx.getLeftY()) > 0.03 || Math.abs(gamepadEx.getRightX()) > 0.03) {
                drive.setGamepad(true);
                drive.moveRobotFieldRelative(gamepadEx.getLeftY(), gamepadEx.getLeftX(), gamepadEx.getRightX());
            }
            else {
                drive.setGamepad(false);
            }
        }
    }
}
