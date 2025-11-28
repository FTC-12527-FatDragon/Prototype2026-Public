package org.firstinspires.ftc.teamcode.subsystems.shooter;

import static org.firstinspires.ftc.teamcode.subsystems.shooter.ShooterConstants.releaseVelocity;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.utils.Util;

public class Shooter extends SubsystemBase {
    public final DcMotorEx leftShooter;
    public final DcMotorEx rightShooter;
    public final TelemetryPacket packet = new TelemetryPacket();
    public static double balls = 0;
    public static boolean readyToShoot = false;

    public final PIDController pidController;
    public static double shooterOpenLoopPower = -1;

    public ShooterState shooterState = ShooterState.STOP;

    public Shooter(final HardwareMap hardwareMap) {
        leftShooter = hardwareMap.get(DcMotorEx.class, ShooterConstants.leftShooterName);
        rightShooter = hardwareMap.get(DcMotorEx.class, ShooterConstants.rightShooterName);
        pidController = new PIDController(ShooterConstants.kP,
                ShooterConstants.kI, ShooterConstants.kD);
    }

    public enum ShooterState {
        STOP(ShooterConstants.stopVelocity),
        SLOW(ShooterConstants.slowVelocity),
        FAST(ShooterConstants.fastVelocity),
        OPENLOOP(0);

        final double shooterVelocity;

        ShooterState(double shooterVelocity) {
            this.shooterVelocity = shooterVelocity;
        }
    }

    public void toggleShooterState(ShooterState shooterStateE) {
        if (shooterStateE == ShooterState.SLOW) {
            shooterState = shooterState == ShooterState.STOP ? ShooterState.SLOW : ShooterState.STOP;
        }
        else if (shooterStateE == ShooterState.FAST) {
            shooterState = shooterState == ShooterState.STOP ? ShooterState.FAST : ShooterState.STOP;
        }
    }

    public void setOpenLoopPower(double power) {
        shooterState = ShooterState.OPENLOOP;
        shooterOpenLoopPower = power;
    }

    public void setShooterState(ShooterState state) {
        shooterState = state;
    }

    public double getVelocity() {
        return rightShooter.getVelocity();
    }

    public boolean isShooterAtSetPoint() {
        return Util.epsilonEqual(shooterState.shooterVelocity,
                rightShooter.getVelocity(), ShooterConstants.shooterEpsilon);
    }

    public double getBalls() {
        return balls;
    }

    public void setBalls(double ballNumber) {
        balls = ballNumber;
    }

    @Override
    public void periodic() {
        if (shooterState != ShooterState.OPENLOOP) {
            if (shooterState != ShooterState.STOP) {
                double currentPower = pidController.calculate(
                        rightShooter.getVelocity(), shooterState.shooterVelocity);
                leftShooter.setPower(-currentPower);
                rightShooter.setPower(currentPower);
                packet.put("currentPower", currentPower);
            }
            else {
                leftShooter.setPower(-ShooterState.STOP.shooterVelocity);
                rightShooter.setPower(ShooterState.STOP.shooterVelocity);
            }
        }
        else {
            leftShooter.setPower(-shooterOpenLoopPower);
            rightShooter.setPower(shooterOpenLoopPower);
        }
        if (isShooterAtSetPoint()) {
            readyToShoot = true;
        }
        else if (rightShooter.getVelocity() <= releaseVelocity) {
            if (readyToShoot) balls = balls >= 1? balls - 1: balls;
            readyToShoot = false;
        }
        packet.put("leftShooterVelocity", leftShooter.getVelocity());
        packet.put("rightShooterVelocity", rightShooter.getVelocity());
        FtcDashboard.getInstance().sendTelemetryPacket(packet);
    }
}
