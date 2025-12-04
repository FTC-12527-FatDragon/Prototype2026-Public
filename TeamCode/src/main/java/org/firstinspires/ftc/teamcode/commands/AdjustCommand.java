package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.cds.CDS;
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake;
import org.firstinspires.ftc.teamcode.subsystems.transit.Transit;

public class AdjustCommand extends CommandBase {
    private final Transit transit;
    private final Intake intake;
    private final CDS cds;

    public AdjustCommand(Transit transit, Intake intake, CDS cds) {
        this.transit = transit;
        this.intake = intake;
        this.cds = cds;
    }

    @Override
    public void initialize() {
        transit.setChooseServoState(Transit.ChooseServoState.OPEN);
        transit.setLimitServoState(Transit.LimitServoState.OPEN);
//        intake.reverseMotor(true);
    }

    @Override
    public void execute() {
        if (cds.getFirst() == 0) {
            transit.setTransitState(Transit.TransitState.SHOOT);
//            if(!intake.isRunning()) intake.toggle();
        }
    }

    @Override
    public void end(boolean interrupted) {
        transit.setTransitState(Transit.TransitState.STOP);
        transit.setChooseServoState(Transit.ChooseServoState.CLOSE);
        transit.setLimitServoState(Transit.LimitServoState.CLOSE);
    }

}
