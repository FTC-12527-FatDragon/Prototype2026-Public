package org.firstinspires.ftc.teamcode.subsystems.shooter;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.utils.DcMotorRe;

public class Shooter extends SubsystemBase {
    public final DcMotorRe shooter;

    public enum ShooterState {
        STOP(ShooterConstants.stopPower),
        SLOW(ShooterConstants.slowPower),
        FAST(ShooterConstants.fastPower);

        double shooterPower;

        ShooterState(double shooterPower) {
            this.shooterPower = shooterPower;
        }
    }

    public ShooterState shooterState = ShooterState.STOP;

    public Shooter(final HardwareMap hardwareMap) {
        shooter = new DcMotorRe(hardwareMap, "shooterMotor");
    }

    public double getAverageVelocity() {
        return shooter.getAverageVelocity();
    }

    public double getInstantVelocity() {
        return shooter.getInstantVelocity();
    }

    public void toggleShooterState(ShooterState shooterStateE) {
        if (shooterStateE == ShooterState.SLOW) {
            shooterState = shooterState == ShooterState.STOP ? ShooterState.SLOW : ShooterState.STOP;
        }
        else if (shooterStateE == ShooterState.FAST) {
            shooterState = shooterState == ShooterState.STOP ? ShooterState.FAST : ShooterState.STOP;
        }
    }

//    public void setShooterVelocity(double setPoint) {
//
//    }
//
//    public boolean isShooterAtSetPoint(double setPoint) {
//        return Math.abs(shooterVelocity - setPoint) <= ShooterConstants.shooterEpsilon;
//    }

    @Override
    public void periodic() {
        shooter.setPower(shooterState.shooterPower);
        shooter.updateLastPos();
    }
}
