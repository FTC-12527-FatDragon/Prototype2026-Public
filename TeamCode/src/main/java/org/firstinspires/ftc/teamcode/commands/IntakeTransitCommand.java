package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.intake.Intake;
import org.firstinspires.ftc.teamcode.subsystems.transit.Transit;
import org.firstinspires.ftc.teamcode.subsystems.vision.CDS;

public class IntakeTransitCommand extends CommandBase {
    private final Transit transit;

    private final CDS cds;

    public IntakeTransitCommand(Transit transit, CDS cds) {
        this.transit = transit;
        this.cds = cds;
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {

    }

    @Override
    public void end(boolean interrupted) {

    }
}
