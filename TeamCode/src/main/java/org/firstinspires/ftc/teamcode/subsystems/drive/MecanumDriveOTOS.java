package org.firstinspires.ftc.teamcode.subsystems.drive;

import static org.firstinspires.ftc.teamcode.subsystems.drive.DriveConstants.strafingBalance;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.utils.Util;

@Config
public class MecanumDriveOTOS extends SubsystemBase {
    private final DcMotor leftFrontMotor, leftBackMotor, rightFrontMotor, rightBackMotor;

    private final SparkFunOTOS otos;
    private double yawOffset;// mm

    public MecanumDriveOTOS(final HardwareMap hardwareMap) {
        leftFrontMotor = hardwareMap.get(DcMotor.class, "leftFrontMotor");
        leftBackMotor = hardwareMap.get(DcMotor.class, "leftBackMotor");
        rightFrontMotor = hardwareMap.get(DcMotor.class, "rightFrontMotor");
        rightBackMotor = hardwareMap.get(DcMotor.class, "rightBackMotor");
        otos = hardwareMap.get(SparkFunOTOS.class, "otos");

        leftFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        otos.resetTracking();
        otos.setAngularUnit(DriveConstants.angleUnit);
        otos.setLinearUnit(DriveConstants.distanceUnit);
        otos.setOffset(new SparkFunOTOS.Pose2D(DriveConstants.xPoseOTOS,
                DriveConstants.yPoseOTOS, DriveConstants.headingPoseOTOS));

        leftFrontMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBackMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void stop() {
        moveRobot(0, 0, 0);
    }

    public void reset(double heading) {
        yawOffset = otos.getPosition().h + heading;
    }

    public void moveRobotFieldRelative(double forward, double fun, double turn) {

        double botHeading = otos.getPosition().h - yawOffset;
        // Rotate the movement direction counter to the bot's rotation\\
        double rotX = fun * Math.cos(-botHeading) - forward * Math.sin(-botHeading);
        double rotY = fun * Math.sin(-botHeading) + forward * Math.cos(-botHeading);

        rotX = rotX * strafingBalance; // Counteract imperfect strafing

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(turn), 1);
        double leftFrontPower = (rotY + rotX + turn) / denominator;
        double leftBackPower = (rotY - rotX + turn) / denominator;
        double rightFrontPower = (rotY - rotX - turn) / denominator;
        double rightBackPower = (rotY + rotX - turn) / denominator;

        leftFrontMotor.setPower(leftFrontPower);
        leftBackMotor.setPower(leftBackPower);
        rightFrontMotor.setPower(rightFrontPower);
        rightBackMotor.setPower(rightBackPower);
    }

    public void moveRobot(double forward, double fun, double turn) {
        double rotX = fun * strafingBalance; // Counteract imperfect strafing
        double rotY = forward;

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(turn), 1);
        double leftFrontPower = (rotY + rotX + turn) / denominator;
        double leftBackPower = (rotY - rotX + turn) / denominator;
        double rightFrontPower = (rotY - rotX - turn) / denominator;
        double rightBackPower = (rotY + rotX - turn) / denominator;

        leftFrontMotor.setPower(leftFrontPower);
        leftBackMotor.setPower(leftBackPower);
        rightFrontMotor.setPower(rightFrontPower);
        rightBackMotor.setPower(rightBackPower);
    }

    public void turnRobot(double angle, double power) {
        double preAngle = otos.getPosition().h;
        while (otos.getPosition().h - preAngle < angle) {
            leftFrontMotor.setPower(power * 0.2);
            leftBackMotor.setPower(power * 0.2);
            rightFrontMotor.setPower(power * -0.2);
            rightBackMotor.setPower(power * -0.2);
        }
    }

    public void turnRobotTo(double angle, double power) {
        double heading = otos.getPosition().h;
        double needs = (angle - heading) % (2 * Math.PI);
        while (Util.epsilonEqual(angle,otos.getPosition().h,0.02)) {
            leftFrontMotor.setPower(power * 0.2);
            leftBackMotor.setPower(power * 0.2);
            rightFrontMotor.setPower(power * -0.2);
            rightBackMotor.setPower(power * -0.2);
        }
    }

    public Pose2D getPose() {
        SparkFunOTOS.Pose2D pose = otos.getPosition();
        return new Pose2D(DriveConstants.distanceUnit, pose.x, pose.y, DriveConstants.angleUnit, pose.h);
    }

    public double getYawOffset() {return yawOffset;}

    public boolean isHeadingAtSetPoint(double headingSetPoint) {
        return Util.epsilonEqual(otos.getPosition().h, headingSetPoint,
                DriveConstants.headingEpsilon);
    }

    
}
