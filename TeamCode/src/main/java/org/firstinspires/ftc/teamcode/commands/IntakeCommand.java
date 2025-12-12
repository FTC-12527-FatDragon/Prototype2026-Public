package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.subsystems.cds.CDS;
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake;
import org.firstinspires.ftc.teamcode.subsystems.transit.Transit;

public class IntakeCommand extends CommandBase {
    private final Transit transit;
    private final Intake intake;
    private final CDS cds;

    public IntakeCommand(Transit transit, Intake intake, CDS cds) {
        this.transit = transit;
        this.intake = intake;
        this.cds = cds;
    }

    @Override
    public void initialize() {
        intake.reverseMotor(false);
        transit.setLimitServoState(Transit.LimitServoState.CLOSE);
    }

    @Override
    public void execute() {
        if (!intake.isRunning()) intake.toggle();
        if (intake.getShooting()) intake.toogleShooting();
        intake.reverseMotor(cds.getBallNum() >= 3);

        transit.setTransitState(Transit.TransitState.INTAKE);
    }

    @Override
    public void end(boolean interrupted) {
        if (intake.isRunning()) intake.toggle();

        transit.setTransitState(Transit.TransitState.STOP);
    }
}
