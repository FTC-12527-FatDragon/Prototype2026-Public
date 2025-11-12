package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.subsystems.intake.Intake;
import org.firstinspires.ftc.teamcode.subsystems.transit.Transit;

public class IntakeCommand extends CommandBase {
    private final Transit transit;
    private final Intake intake;

    public IntakeCommand(Transit transit, Intake intake) {
        this.transit = transit;
        this.intake = intake;
    }

    @Override
    public void initialize() {
        intake.reverseMotor(false);
        transit.setLimitServoState(Transit.LimitServoState.CLOSE);
    }

    @Override
    public void execute() {
        if (!intake.isRunning()) {
            intake.toggle();
        }

        transit.setTransitState(Transit.TransitState.INTAKE);
    }

    @Override
    public void end(boolean interrupted) {
        if (intake.isRunning()) {
            intake.toggle();
        }

        transit.setTransitState(Transit.TransitState.STOP);
    }
}
