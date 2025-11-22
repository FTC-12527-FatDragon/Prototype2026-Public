package org.firstinspires.ftc.teamcode.commands.autoCommands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.intake.Intake;
import org.firstinspires.ftc.teamcode.subsystems.shooter.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.transit.Transit;

public class AutoShootCommand extends CommandBase {
    private final Transit transit;
    private final Intake intake;
    private final Shooter shooter;

    public AutoShootCommand(Transit transit, Intake intake, Shooter shooter) {
        this.transit = transit;
        this.intake = intake;
        this.shooter = shooter;
    }

    @Override
    public void execute() {
        if (transit.chooseState == Transit.ChooseServoState.CLOSE) {
            if (shooter.isShooterAtSetPoint() && shooter.shooterState != Shooter.ShooterState.STOP) {
                transit.setTransitState(Transit.TransitState.SHOOT);
                transit.setLimitServoState(Transit.LimitServoState.OPEN);
            }
            else {
                transit.setTransitState(Transit.TransitState.STOP);
                transit.setLimitServoState(Transit.LimitServoState.CLOSE);
            }
        }
        if (!intake.isRunning()) intake.toggle();

    }

    @Override
    public void end(boolean interrupted) {
        transit.setTransitState(Transit.TransitState.STOP);
        if (intake.isRunning()) intake.toggle();
    }
}
