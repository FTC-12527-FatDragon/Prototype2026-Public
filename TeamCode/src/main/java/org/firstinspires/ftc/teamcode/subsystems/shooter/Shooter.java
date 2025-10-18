package org.firstinspires.ftc.teamcode.subsystems.shooter;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Shooter extends SubsystemBase {
    public final Motor leftShooter, rightShooter, transit;

    public static double shooterVelocity;

    public Shooter(final HardwareMap hardwareMap) {
        leftShooter = new MotorEx(hardwareMap, ShooterConstants.leftShooterName, Motor.GoBILDA.RPM_312);
        rightShooter = new MotorEx(hardwareMap, ShooterConstants.rightShooterName, Motor.GoBILDA.RPM_312);
        transit = new Motor(hardwareMap, ShooterConstants.transit);

        leftShooter.setRunMode(Motor.RunMode.VelocityControl);
        leftShooter.setVeloCoefficients(ShooterConstants.kP, ShooterConstants.kI, ShooterConstants.kD);
        transit.setRunMode(Motor.RunMode.RawPower);

        shooterVelocity = 0;
    }

    public double getShooterVelocity() {
        return leftShooter.getCorrectedVelocity();
    }

    public void setShooterVelocity(double setPoint) {
        shooterVelocity = setPoint;
    }

    public boolean isShooterAtSetPoint(double setPoint) {
        return Math.abs(shooterVelocity - setPoint) <= ShooterConstants.shooterEpsilon;
    }

    public void setTransitPower(double power) {
        transit.set(power);
    }

    @Override
    public void periodic() {
//        leftShooter.setVelocity(shooterVelocity);
//        rightShooter.setVelocity(-shooterVelocity);
    }
}
