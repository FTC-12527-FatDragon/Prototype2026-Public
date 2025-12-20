package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.cds.CDS;
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake;
import org.firstinspires.ftc.teamcode.subsystems.transit.Transit;

public class JudgeTransitCommand extends CommandBase {
    private final Transit transit;
    private final Intake intake;
    private final CDS cds;

    public JudgeTransitCommand(Transit transit, Intake intake,
                          CDS cds) {
        this.transit = transit;
        this.intake = intake;
        this.cds = cds;
    }

    @Override
    public void execute() {
        if (!intake.getShooting()) intake.toogleShooting();
        if (transit.chooseState == Transit.ChooseServoState.CLOSE) {
                transit.setTransitState(Transit.TransitState.SHOOT);
                transit.setLimitServoState(Transit.LimitServoState.OPEN);
//            if (shooter.isShooterAtSetPoint() && shooter.shooterState != Shooter.ShooterState.STOP) {
//                transit.setTransitState(Transit.TransitState.SHOOT);
//                transit.setLimitServoState(Transit.LimitServoState.OPEN);
//            }
//            else {
//                transit.setTransitState(Transit.TransitState.STOP);
//                transit.setLimitServoState(Transit.LimitServoState.CLOSE);
//            }
        }
        if (!intake.isRunning()) intake.toggle();

    }

    @Override
    public void end(boolean interrupted) {
        transit.setTransitState(Transit.TransitState.STOP);
        if (intake.isRunning()) intake.toggle();
        if (intake.getShooting()) intake.toogleShooting();
        cds.deleteBalls();
    }
}
