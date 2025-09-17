package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.shooter.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.shooter.ShooterConstants;

public class ShootCommand extends CommandBase {
    private final Shooter shooter;
    private final double shooterSetpoint;

    public ShootCommand(Shooter shooter, double shooterSetpoint) {
        this.shooter = shooter;
        this.shooterSetpoint = shooterSetpoint;
    }

    @Override
    public void execute() {
        shooter.setShooterVelocity(shooterSetpoint);

        if (shooter.isShooterAtSetPoint(shooterSetpoint)) {
            shooter.setTransitPower(ShooterConstants.transitShootPower);
        }
    }

    @Override
    public void end(boolean interrupted) {
        shooter.setShooterVelocity(0);
    }
}
