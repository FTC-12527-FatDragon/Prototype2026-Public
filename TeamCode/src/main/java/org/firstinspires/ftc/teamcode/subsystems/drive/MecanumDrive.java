package org.firstinspires.ftc.teamcode.subsystems.drive;

import static org.firstinspires.ftc.teamcode.subsystems.drive.DriveConstants.strafingBalance;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.utils.Units;
import org.firstinspires.ftc.teamcode.utils.Util;

@Config
public class MecanumDrive extends SubsystemBase {
    private final DcMotor leftFrontMotor, leftBackMotor, rightFrontMotor, rightBackMotor;
    private final GoBildaPinpointDriver od;
    private double yawOffset;
    public static double xPose = DriveConstants.xPose, yPose = DriveConstants.yPose; // mm

    public MecanumDrive(final HardwareMap hardwareMap) {
        leftFrontMotor = hardwareMap.get(DcMotor.class, "leftFrontMotor");
        leftBackMotor = hardwareMap.get(DcMotor.class, "leftBackMotor");
        rightFrontMotor = hardwareMap.get(DcMotor.class, "rightFrontMotor");
        rightBackMotor = hardwareMap.get(DcMotor.class, "rightBackMotor");
        od = hardwareMap.get(GoBildaPinpointDriver.class, "od");

        leftFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        od.resetPosAndIMU();
        od.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD);
        od.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        od.setOffsets(Units.mmToInches(xPose), Units.mmToInches(yPose), DistanceUnit.INCH);

        leftFrontMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBackMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void reset(double heading) {
        yawOffset = od.getHeading(AngleUnit.RADIANS) + heading;
    }

    public void moveRobotFieldRelative(double forward, double fun, double turn) {
        od.update();

        double botHeading = od.getHeading(AngleUnit.RADIANS) - yawOffset;
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
        double preAngle = od.getHeading(AngleUnit.RADIANS);
        while (od.getHeading(AngleUnit.RADIANS) - preAngle < angle) {
            leftFrontMotor.setPower(power * 0.2);
            leftBackMotor.setPower(power * 0.2);
            rightFrontMotor.setPower(power * -0.2);
            rightBackMotor.setPower(power * -0.2);
        }
    }

    public void turnRobotTo(double angle, double power) {
        double heading = od.getHeading(AngleUnit.RADIANS);
        double needs = (angle - heading) % (2 * Math.PI);
        if(0 <= needs && needs <= Math.PI || needs <= -Math.PI) {
            while (Util.epsilonEqual(angle, od.getHeading(AngleUnit.RADIANS), 0.02)) {
                leftFrontMotor.setPower(power * 0.2);
                leftBackMotor.setPower(power * 0.2);
                rightFrontMotor.setPower(power * -0.2);
                rightBackMotor.setPower(power * -0.2);
            }
        }
    }

    public Pose2D getPose() {
        return od.getPosition();
    }

    public double getYawOffset() {return yawOffset;}

    public boolean isHeadingAtSetPoint(double headingSetPoint) {
        return Util.epsilonEqual(od.getHeading(AngleUnit.RADIANS), headingSetPoint,
                DriveConstants.headingEpsilon);
    }

    public void stop() {
        moveRobot(0, 0, 0);
    }
}
