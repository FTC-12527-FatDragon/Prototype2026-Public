package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.cds.CDS;
import org.firstinspires.ftc.teamcode.subsystems.drive.VisionMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake;
import org.firstinspires.ftc.teamcode.subsystems.shooter.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.transit.Transit;

public class TransitCommand extends CommandBase {
    private final Transit transit;
    private final Intake intake;
    private final Shooter shooter;
    private final CDS cds;
    private final VisionMecanumDrive drive;

    public TransitCommand(Transit transit, Intake intake, Shooter shooter,
                          CDS cds) {
        this.transit = transit;
        this.intake = intake;
        this.shooter = shooter;
        this.cds = cds;
        this.drive = null;
    }

    public TransitCommand(Transit transit, Intake intake, Shooter shooter,
                          CDS cds, VisionMecanumDrive drive) {
        this.transit = transit;
        this.intake = intake;
        this.shooter = shooter;
        this.cds = cds;
        this.drive = drive;
    }

    @Override
    public void execute() {
        if (!intake.getShooting()) intake.toogleShooting();
        if (shooter.isShooterAtSetPoint() && shooter.shooterState != Shooter.ShooterState.STOP) {
            transit.setTransitState(Transit.TransitState.SHOOT);
            transit.setLimitServoState(Transit.LimitServoState.OPEN);
        }
//            if (shooter.isShooterAtSetPoint() && shooter.shooterState != Shooter.ShooterState.STOP) {
//                transit.setTransitState(Transit.TransitState.SHOOT);
//                transit.setLimitServoState(Transit.LimitServoState.OPEN);
//            }
//            else {
//                transit.setTransitState(Transit.TransitState.STOP);
//                transit.setLimitServoState(Transit.LimitServoState.CLOSE);
//            }
        if (!intake.isRunning()) intake.toggle();
        if (drive != null) drive.visionCalibrate();
    }

    @Override
    public void end(boolean interrupted) {
        transit.setTransitState(Transit.TransitState.STOP);
        if (intake.isRunning()) intake.toggle();
        if (intake.getShooting()) intake.toogleShooting();
        cds.deleteBalls();
    }
}
