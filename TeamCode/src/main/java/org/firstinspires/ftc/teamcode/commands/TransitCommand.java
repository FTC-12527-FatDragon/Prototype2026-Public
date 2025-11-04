package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.transit.Transit;

public class TransitCommand extends CommandBase {
    private final Transit transit;

    public TransitCommand(Transit transit) {
        this.transit = transit;
    }

    @Override
    public void execute() {
        transit.setTransitState(Transit.TransitState.SHOOT);
    }

    @Override
    public void end(boolean interrupted) {
        transit.setTransitState(Transit.TransitState.STOP);
    }
}
