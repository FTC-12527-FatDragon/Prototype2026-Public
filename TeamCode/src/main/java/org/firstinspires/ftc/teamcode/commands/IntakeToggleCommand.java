package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.intake.Intake;

public class IntakeToggleCommand extends CommandBase {
    private final Intake intake;

    public IntakeToggleCommand(Intake intake) {
        this.intake = intake;
    }

    @Override
    public void execute() {
        intake.toggle();
    }

    @Override
    public void end(boolean interrupted) {
        if(intake.isRunning()) intake.toggle();
    }
}
