package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;

import org.firstinspires.ftc.teamcode.subsystems.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.drive.MecanumDriveOTOS;

import java.util.function.BooleanSupplier;

public class TeleOpDriveCommand extends CommandBase {
    private final MecanumDriveOTOS drive;
    private final GamepadEx gamepadEx;
    private final BooleanSupplier isAlign;

    public TeleOpDriveCommand(MecanumDriveOTOS drive, GamepadEx gamepadEx, BooleanSupplier isAlign) {
        this.drive = drive;
        this.gamepadEx = gamepadEx;
        this.isAlign = isAlign;
        addRequirements(drive);
    }

    @Override
    public void execute() {
        if (!isAlign.getAsBoolean()) {
            if (Math.abs(gamepadEx.getLeftX()) > 0.03 || Math.abs(gamepadEx.getLeftY()) > 0.03 || Math.abs(gamepadEx.getRightX()) > 0.03) {
                drive.setDriveState(MecanumDriveOTOS.DriveState.TELEOP);
                drive.moveRobotFieldRelative(gamepadEx.getLeftY(), gamepadEx.getLeftX(), gamepadEx.getRightX());
            }
            else{
                drive.setDriveState(MecanumDriveOTOS.DriveState.STOP);
            }
        }
        else {
            drive.setDriveState(MecanumDriveOTOS.DriveState.ALIGN);
        }
    }
}
