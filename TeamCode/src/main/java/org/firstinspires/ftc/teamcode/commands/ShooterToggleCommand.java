package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.shooter.Shooter;

public class ShooterToggleCommand extends CommandBase {
    private final Shooter shooter;

    public ShooterToggleCommand(Shooter shooter) {
        this.shooter = shooter;
    }

    @Override
    public void execute() {
        shooter.
    }

    @Override
    public void end(boolean interrupted) {
        shooter.setShooterVelocity(0);
    }
}
