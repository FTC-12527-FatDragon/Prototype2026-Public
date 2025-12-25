package org.firstinspires.ftc.teamcode.subsystems.shooter;

import static org.firstinspires.ftc.teamcode.subsystems.shooter.ShooterConstants.releaseVelocity;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.utils.Util;

public class Shooter extends SubsystemBase {
    public final DcMotorEx leftShooter;
    public final DcMotorEx rightShooter;
    public final Servo brakeServo;
    public final TelemetryPacket packet = new TelemetryPacket();

    public ShooterState shooterState = ShooterState.STOP;

    public final boolean highSpeed;

    public Shooter(final HardwareMap hardwareMap, boolean highSpeed) {
        leftShooter = hardwareMap.get(DcMotorEx.class, ShooterConstants.leftShooterName);
        rightShooter = hardwareMap.get(DcMotorEx.class, ShooterConstants.rightShooterName);
        brakeServo = hardwareMap.get(Servo.class, ShooterConstants.brakeServoName);
        this.highSpeed = highSpeed;
    }

    public enum ShooterState {
        STOP(ShooterConstants.stopVelocity),
        FASTSTOP(ShooterConstants.fastStopVelocity),
        SLOW(ShooterConstants.slowVelocity),
        FAST(ShooterConstants.fastVelocity),
        OPENLOOP(0);

        final double shooterVelocity;

        ShooterState(double shooterVelocity) {
            this.shooterVelocity = shooterVelocity;
        }
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

    public void brakeShooter() {
        brakeServo.setPosition(ShooterConstants.brakePose);
    }

    public void releaseShooter() {
        brakeServo.setPosition(ShooterConstants.releasePose);
    }

    @Override
    public void periodic() {
//        if (shooterState != ShooterState.STOP) {
//            double currentPower = pidController.calculate(
//                    rightShooter.getVelocity(), shooterState.shooterVelocity);
//            leftShooter.setPower(-currentPower);
//            rightShooter.setPower(currentPower);
//            packet.put("currentPower", currentPower);
//        }
        if (shooterState != ShooterState.STOP) {
            releaseShooter();
            if (shooterState == ShooterState.FAST) {
                leftShooter.setPower(-0.95);
                rightShooter.setPower(0.95);
            }
            else {
                leftShooter.setPower(-0.8);
                rightShooter.setPower(0.8);
            }
        }
        else {
            if (highSpeed && getVelocity() > ShooterState.FAST.shooterVelocity) {
                brakeShooter();
            }
            else if (!highSpeed && getVelocity() > ShooterState.SLOW.shooterVelocity) {
                brakeShooter();
            }
            else releaseShooter();
            if (!highSpeed) {
                leftShooter.setPower(-ShooterState.STOP.shooterVelocity);
                rightShooter.setPower(ShooterState.STOP.shooterVelocity);
            }
            else {
                leftShooter.setPower(-ShooterState.FASTSTOP.shooterVelocity);
                rightShooter.setPower(ShooterState.FASTSTOP.shooterVelocity);
            }
        }

        packet.put("shooterVelocity", rightShooter.getVelocity());
        packet.put("shooterPower", rightShooter.getPower());
        FtcDashboard.getInstance().sendTelemetryPacket(packet);
    }
}
