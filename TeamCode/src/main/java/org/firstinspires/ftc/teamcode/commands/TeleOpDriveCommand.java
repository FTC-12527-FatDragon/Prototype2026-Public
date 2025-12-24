package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;

import org.firstinspires.ftc.teamcode.subsystems.drive.VisionMecanumDrive;

import java.util.function.BooleanSupplier;

public class TeleOpDriveCommand extends CommandBase {
    private final VisionMecanumDrive drive;
    private final GamepadEx gamepadEx;
    private final BooleanSupplier isAlign;

    public TeleOpDriveCommand(VisionMecanumDrive drive, GamepadEx gamepadEx, BooleanSupplier isAlign) {
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
                drive.setDriveState(VisionMecanumDrive.DriveState.TELEOP);
                drive.moveRobotFieldRelative(gamepadEx.getLeftY(),
                        gamepadEx.getLeftX(), gamepadEx.getRightX());
            }
            else{
                drive.setDriveState(VisionMecanumDrive.DriveState.STOP);
            }
        }
        else {
            drive.setDriveState(VisionMecanumDrive.DriveState.ALIGN);
            drive.moveRobotFieldRelative(gamepadEx.getLeftY(),
                    gamepadEx.getLeftX(), drive.getAlignTurnPower());
        }
    }
}
