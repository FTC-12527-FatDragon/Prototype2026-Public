package org.firstinspires.ftc.teamcode.subsystems.shooter;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.utils.DcMotorRe;
import org.firstinspires.ftc.teamcode.utils.Util;

public class Shooter extends SubsystemBase {
    public final DcMotorRe leftShooter;
    public final DcMotorRe rightShooter;
    public final
    TelemetryPacket packet = new TelemetryPacket();

    public final PIDController pidController;
    public static double shooterOpenLoopPower = -1;

    public ShooterState shooterState = ShooterState.STOP;

    public Shooter(final HardwareMap hardwareMap) {
        leftShooter = new DcMotorRe(hardwareMap, ShooterConstants.leftShooterName);
        rightShooter = new DcMotorRe(hardwareMap, ShooterConstants.rightShooterName);
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

    public double getAverageVelocity() {
        return rightShooter.getAverageVelocity();
    }

    public double getInstantVelocity() {
        return rightShooter.getInstantVelocity();
    }

    public boolean isShooterAtSetPoint() {
        return Util.epsilonEqual(shooterState.shooterVelocity,
                rightShooter.getInstantVelocity(), ShooterConstants.shooterEpsilon);
    }

    @Override
    public void periodic() {
        if (shooterState != ShooterState.OPENLOOP) {
            if (shooterState != ShooterState.STOP) {
                double currentPower = pidController.calculate(
                        rightShooter.getInstantVelocity(), shooterState.shooterVelocity);
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
        rightShooter.updateLastPos();
        packet.put("leftShooterVelocity", leftShooter.getInstantVelocity());
        packet.put("rightShooterVelocity", rightShooter.getInstantVelocity());
        FtcDashboard.getInstance().sendTelemetryPacket(packet);
    }
}
