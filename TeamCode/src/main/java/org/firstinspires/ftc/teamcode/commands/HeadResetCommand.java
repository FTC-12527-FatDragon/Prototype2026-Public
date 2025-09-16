package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.drive.Drive;

public class HeadResetCommand extends CommandBase {
    private final Drive drive;

    public HeadResetCommand(Drive drive) {
        this.drive = drive;
    }

    @Override
    public void execute() {
        drive.resetHead();
    }

}
