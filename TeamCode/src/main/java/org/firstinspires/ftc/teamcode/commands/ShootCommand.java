package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.shooter.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.shooter.ShooterConstants;

public class ShootCommand extends CommandBase {
    private final Shooter shooter;
    private final double shooterSetpoint;
    private final MecanumDrive drive;
    private final double headingSetPoint;

    public ShootCommand(Shooter shooter, double shooterSetpoint,
                        MecanumDrive drive, double headingSetPoint) {
        this.shooter = shooter;
        this.shooterSetpoint = shooterSetpoint;
        this.drive = drive;
        this.headingSetPoint = headingSetPoint;
    }

    @Override
    public void execute() {
        shooter.setShooterVelocity(shooterSetpoint);
        drive.turnRobotTo(headingSetPoint, 0.8);

        if (shooter.isShooterAtSetPoint(shooterSetpoint)
                && drive.isHeadingAtSetPoint(headingSetPoint)) {
            shooter.setTransitPower(ShooterConstants.transitShootPower);
        }
    }

    @Override
    public void end(boolean interrupted) {
        shooter.setShooterVelocity(0);
    }
}
