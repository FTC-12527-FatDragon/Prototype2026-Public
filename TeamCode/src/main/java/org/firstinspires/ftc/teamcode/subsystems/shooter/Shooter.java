package org.firstinspires.ftc.teamcode.subsystems.shooter;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.utils.DcMotorRe;

public class Shooter extends SubsystemBase {
    public final DcMotorRe shooter;

    public Shooter(final HardwareMap hardwareMap) {
        shooter = new DcMotorRe(hardwareMap, "shooterMotor");
    }

    public double getAverageVelocity() {
        return shooter.getAverageVelocity();
    }

    public double getInstantVelocity() {
        return shooter.getInstantVelocity();
    }

    public void setPower(double power) {
        shooter.setPower(power);
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
        shooter.updateLastPos();
    }
}
