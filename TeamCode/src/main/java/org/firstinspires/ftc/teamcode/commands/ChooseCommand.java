package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.intake.Intake;
import org.firstinspires.ftc.teamcode.subsystems.shooter.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.transit.Transit;

public class ChooseCommand extends CommandBase {
    private final Transit transit;
    private final Intake intake;

    public ChooseCommand(Transit transit, Intake intake) {
        this.transit = transit;
        this.intake = intake;
    }

    @Override
    public void execute() {
        intake.reverseMotor(true);
        if (transit.chooseState == Transit.ChooseServoState.OPEN) {
            transit.setTransitState(Transit.TransitState.INTAKE);
        }
        if (!intake.isRunning()) intake.toggle();

    }

    @Override
    public void end(boolean interrupted) {
        transit.setTransitState(Transit.TransitState.STOP);
        if (intake.isRunning()) intake.toggle();
    }
}