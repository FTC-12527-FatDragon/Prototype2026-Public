package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;

import org.firstinspires.ftc.teamcode.subsystems.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.drive.MecanumDriveOTOS;

import java.util.function.BooleanSupplier;

public class TeleOpDriveCommand extends CommandBase {
    private final MecanumDrive drive;
    private final GamepadEx gamepadEx;
    private final BooleanSupplier isAlign;

    public TeleOpDriveCommand(MecanumDrive drive, GamepadEx gamepadEx, BooleanSupplier isAlign) {
        this.drive = drive;
        this.gamepadEx = gamepadEx;
        this.isAlign = isAlign;
        addRequirements(drive);
    }

    @Override
    public void execute() {
        if (!isAlign.getAsBoolean()) {
            if (Math.abs(gamepadEx.getLeftX()) > 0.03 || Math.abs(gamepadEx.getLeftY())
                    > 0.03 || Math.abs(gamepadEx.getRightX()) > 0.03) {
                drive.setDriveState(MecanumDrive.DriveState.TELEOP);
                drive.moveRobotFieldRelative(gamepadEx.getLeftY(),
                        gamepadEx.getLeftX(), gamepadEx.getRightX());
            }
            else{
                drive.setDriveState(MecanumDrive.DriveState.STOP);
            }
        }
        else {
            drive.setDriveState(MecanumDrive.DriveState.ALIGN);
            drive.moveRobotFieldRelative(gamepadEx.getLeftY(),
                    gamepadEx.getLeftX(), drive.getAlignTurnPower());
        }
    }
}
