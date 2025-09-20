package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.drive.MecanumDrive;

public class HeadResetCommand extends CommandBase {
    private final MecanumDrive drive;

    public HeadResetCommand(MecanumDrive drive) {
        this.drive = drive;
    }

    @Override
    public void execute() {drive.reset(0);}
}
