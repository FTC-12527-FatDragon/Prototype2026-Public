package org.firstinspires.ftc.teamcode.commands.autoCommands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.shooter.Shooter;

public class AccelerateCommand extends CommandBase {
    private final Shooter shooter;
    private final Shooter.ShooterState state;

    public AccelerateCommand(Shooter shooter, Shooter.ShooterState state) {
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
