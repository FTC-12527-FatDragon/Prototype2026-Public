package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.cds.CDS;
import org.firstinspires.ftc.teamcode.subsystems.drive.VisionMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake;
import org.firstinspires.ftc.teamcode.subsystems.shooter.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.shooter.ShooterConstants;
import org.firstinspires.ftc.teamcode.subsystems.transit.Transit;
import org.firstinspires.ftc.teamcode.utils.Util;

public class TransitCommand extends CommandBase {
    private final Transit transit;
    private final Intake intake;
    private final Shooter shooter;
    private final CDS cds;
    private final VisionMecanumDrive drive;
    private final boolean speedAdjust;

    public TransitCommand(Transit transit, Intake intake, Shooter shooter,
                          CDS cds) {
        this.transit = transit;
        this.intake = intake;
        this.shooter = shooter;
        this.cds = cds;
        this.drive = null;
        this.speedAdjust = false;
    }

    public TransitCommand(Transit transit, Intake intake, Shooter shooter,
                          CDS cds, VisionMecanumDrive drive, boolean speedAdjust) {
        this.transit = transit;
        this.intake = intake;
        this.shooter = shooter;
        this.cds = cds;
        this.drive = drive;
        this.speedAdjust = speedAdjust;
    }

    @Override
    public void execute() {
        if (!intake.getShooting()) intake.toggleShooting();
        if (!speedAdjust) {
            if (shooter.isShooterAtSetPoint() && shooter.shooterState != Shooter.ShooterState.STOP) {
                transit.setTransitState(Transit.TransitState.SHOOT);
                transit.setLimitServoState(Transit.LimitServoState.OPEN);
            }
        }
        else {
            if (drive != null && Util.epsilonEqual(drive.getShooterVelocity(), shooter.getVelocity(),
                    ShooterConstants.shooterEpsilon) && shooter.shooterState == Shooter.ShooterState.DYNAMIC) {
                transit.setTransitState(Transit.TransitState.SHOOT);
                transit.setLimitServoState(Transit.LimitServoState.OPEN);
            }
        }
        if (!intake.isRunning()) intake.toggle();
        if (drive != null) drive.visionCalibrate();
    }

    @Override
    public void end(boolean interrupted) {
        transit.setTransitState(Transit.TransitState.STOP);
        if (intake.isRunning()) intake.toggle();
        if (intake.getShooting()) intake.toggleShooting();
        cds.deleteBalls();
    }
}
