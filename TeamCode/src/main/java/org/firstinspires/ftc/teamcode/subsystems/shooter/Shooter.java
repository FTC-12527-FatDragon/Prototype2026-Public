package org.firstinspires.ftc.teamcode.subsystems.shooter;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Shooter extends SubsystemBase {
    public final DcMotorEx leftShooter, rightShooter;

    public static double shooterVelocity;

    public Shooter(final HardwareMap hardwareMap) {
        leftShooter = hardwareMap.get(DcMotorEx.class, ShooterConstants.leftShooterName);
        rightShooter = hardwareMap.get(DcMotorEx.class, ShooterConstants.leftShooterName);

        shooterVelocity = 0;
    }

    public double getShooterVelocity() {
        return leftShooter.getVelocity();
    }

    public void setShooterVelocity(double setPoint) {
        shooterVelocity = setPoint;
    }

    public boolean isShooterAtSetPoint(double setPoint) {
        return Math.abs(shooterVelocity - setPoint) <= ShooterConstants.shooterEpsilon;
    }

    @Override
    public void periodic() {
//        leftShooter.setVelocity(shooterVelocity);
//        rightShooter.setVelocity(-shooterVelocity);
    }
}
