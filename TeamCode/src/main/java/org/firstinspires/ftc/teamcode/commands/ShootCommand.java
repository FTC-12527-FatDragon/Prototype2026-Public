package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.shooter.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.transit.Transit;
import org.firstinspires.ftc.teamcode.subsystems.transit.TransitConstants;

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
    }

    @Override
    public void end(boolean interrupted) {
        shooter.setShooterVelocity(0);
    }
}
