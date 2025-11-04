package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.shooter.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.shooter.ShooterConstants;
import org.firstinspires.ftc.teamcode.subsystems.transit.Transit;

public class TransitCommandWithShooter extends CommandBase {
    private final Transit transit;
    private final Shooter shooter;

    public TransitCommandWithShooter(Transit transit, Shooter shooter) {
        this.transit = transit;
        this.shooter = shooter;
    }

    @Override
    public void execute() {
        if (shooter.shooterState == Shooter.ShooterState.FAST
                && shooter.getAverageVelocity() > ShooterConstants.fastVelocity
                || shooter.shooterState == Shooter.ShooterState.SLOW
                && shooter.getAverageVelocity() > ShooterConstants.slowVelocity
        ) {
            transit.setTransitState(Transit.TransitState.SHOOT);
            transit.transitServoOn();
        }
        else {
            transit.stopTransit(700);
        }
    }

    @Override
    public void end(boolean interrupted) {
        transit.setTransitState(Transit.TransitState.STOP);
    }
}
