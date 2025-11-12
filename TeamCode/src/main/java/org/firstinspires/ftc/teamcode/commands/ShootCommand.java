package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.shooter.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.transit.Transit;
import org.firstinspires.ftc.teamcode.subsystems.transit.TransitConstants;

public class ShootCommand extends CommandBase {
    private final Shooter shooter;
    private final Shooter.ShooterState state;

    public ShootCommand(Shooter shooter, Shooter.ShooterState state) {
        this.shooter = shooter;
        this.state = state;
    }

    @Override
    public void execute() {
        shooter.setShooterState(state);
    }

    @Override
    public void end(boolean interrupted) {
        shooter.setShooterState(Shooter.ShooterState.STOP);
    }
}
