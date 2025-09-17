package org.firstinspires.ftc.teamcode.subsystems.shooter;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Shooter extends SubsystemBase {
    public final Motor leftShooter, rightShooter, transit;

    public static double shooterVelocity;

    public Shooter(final HardwareMap hardwareMap) {
        leftShooter = new Motor(hardwareMap, ShooterConstants.leftShooterName);
        rightShooter = new Motor(hardwareMap, ShooterConstants.rightShooterName);
        transit = new Motor(hardwareMap, ShooterConstants.transit);

        leftShooter.setRunMode(Motor.RunMode.VelocityControl);
        leftShooter.setVeloCoefficients(ShooterConstants.kP, ShooterConstants.kI, ShooterConstants.kD);
        leftShooter.setFeedforwardCoefficients(ShooterConstants.kS, ShooterConstants.kV, ShooterConstants.kA);

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
        leftShooter.set(shooterVelocity);
        rightShooter.set(-shooterVelocity);
    }
}
