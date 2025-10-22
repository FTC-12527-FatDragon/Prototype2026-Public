package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.transit.Transit;

public class TransitCommand extends CommandBase {
    private final Transit transit;

    private final double power;

    public TransitCommand(Transit transit, double power) {
        this.transit = transit;
        this.power = power;
    }

    @Override
    public void execute() {
        transit.setPower(power);
    }

    @Override
    public void end(boolean interrupted) {
        transit.setPower(0);
    }
}
