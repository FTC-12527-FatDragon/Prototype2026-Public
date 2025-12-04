package org.firstinspires.ftc.teamcode.subsystems.drive;

import static com.arcrobotics.ftclib.purepursuit.PurePursuitUtil.angleWrap;
import static com.qualcomm.robotcore.util.Range.clip;
import static org.firstinspires.ftc.teamcode.subsystems.drive.DriveConstants.farGoalDistance;
import static org.firstinspires.ftc.teamcode.subsystems.drive.DriveConstants.kD_alignH;
import static org.firstinspires.ftc.teamcode.subsystems.drive.DriveConstants.kI_alignH;
import static org.firstinspires.ftc.teamcode.subsystems.drive.DriveConstants.kP_alignH;
import static org.firstinspires.ftc.teamcode.subsystems.drive.DriveConstants.kP_brakeH;
import static org.firstinspires.ftc.teamcode.subsystems.drive.DriveConstants.kP_brakeXY;
import static org.firstinspires.ftc.teamcode.subsystems.drive.DriveConstants.nearGoalDistance;
import static org.firstinspires.ftc.teamcode.subsystems.drive.DriveConstants.strafingBalance;
import static org.firstinspires.ftc.teamcode.subsystems.drive.DriveConstants.xFarPoseBlue;
import static org.firstinspires.ftc.teamcode.subsystems.drive.DriveConstants.xFarPoseRed;
import static org.firstinspires.ftc.teamcode.subsystems.drive.DriveConstants.xNearPoseBlue;
import static org.firstinspires.ftc.teamcode.subsystems.drive.DriveConstants.xNearPoseRed;
import static org.firstinspires.ftc.teamcode.subsystems.drive.DriveConstants.yFarPoseBlue;
import static org.firstinspires.ftc.teamcode.subsystems.drive.DriveConstants.yFarPoseRed;
import static org.firstinspires.ftc.teamcode.subsystems.drive.DriveConstants.yNearPoseBlue;
import static org.firstinspires.ftc.teamcode.subsystems.drive.DriveConstants.yNearPoseRed;
import static org.firstinspires.ftc.teamcode.utils.Util.poseDistance;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.subsystems.shooter.ShooterConstants;
import org.firstinspires.ftc.teamcode.subsystems.vision.AutoApriltag;
import org.firstinspires.ftc.teamcode.utils.Util;

@Config
public class MecanumDriveOTOS extends SubsystemBase {
    public final DcMotor leftFrontMotor, leftBackMotor, rightFrontMotor, rightBackMotor;

    private final SparkFunOTOS otos;
    private final AutoApriltag autoApriltag;
    private double yawOffset;// mm
    private PIDController alignPID;
    private final DriveState alliance;

    public DriveState driveState;

    Pose2D lastPose;

    public enum DriveState {
        STOP,
        TELEOP,
        ALIGN,
        RED,
        BLUE;

        DriveState() {}
    }

    public MecanumDriveOTOS(final HardwareMap hardwareMap, DriveState alliance) {
        leftFrontMotor = hardwareMap.get(DcMotor.class, "leftFrontMotor");
        leftBackMotor = hardwareMap.get(DcMotor.class, "leftBackMotor");
        rightFrontMotor = hardwareMap.get(DcMotor.class, "rightFrontMotor");
        rightBackMotor = hardwareMap.get(DcMotor.class, "rightBackMotor");
        otos = hardwareMap.get(SparkFunOTOS.class, "otos");
        driveState = DriveState.STOP;
        alignPID = new PIDController(kP_alignH, kI_alignH, kD_alignH);
        this.alliance = alliance;

        leftFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        autoApriltag = new AutoApriltag(hardwareMap);

        otos.resetTracking();
        otos.setAngularUnit(DriveConstants.angleUnit);
        otos.setLinearUnit(DriveConstants.distanceUnit);
        otos.setOffset(new SparkFunOTOS.Pose2D(DriveConstants.xPoseOTOS,
                DriveConstants.yPoseOTOS, DriveConstants.headingPoseOTOS));

        leftFrontMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBackMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        lastPose = new Pose2D(DriveConstants.distanceUnit, 0, 0, DriveConstants.angleUnit, 0);
    }

    public void stop() {
        moveRobot(0, 0, 0);
    }

    public void reset(double heading) {
        yawOffset = otos.getPosition().h + heading;
    }

    public void setDriveState(DriveState driveState) {
        this.driveState = driveState;
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

    public Pose2D getPose() {
        SparkFunOTOS.Pose2D pose = otos.getPosition();
        return new Pose2D(DriveConstants.distanceUnit, pose.x, pose.y, DriveConstants.angleUnit, pose.h);
    }

    public double getYawOffset() {return yawOffset;}

    public boolean isHeadingAtSetPoint(double headingSetPoint) {
        return Util.epsilonEqual(otos.getPosition().h, headingSetPoint,
                DriveConstants.headingEpsilon);
    }

    private void applyBrake() {
        Pose2D p = getPose();

        double errorX = lastPose.getX(DriveConstants.distanceUnit) - p.getX(DriveConstants.distanceUnit);
        double errorY = lastPose.getY(DriveConstants.distanceUnit) - p.getY(DriveConstants.distanceUnit);
        double errorH = angleWrap(lastPose.getHeading(DriveConstants.angleUnit) - p.getHeading(DriveConstants.angleUnit));

        double forward = errorY * kP_brakeXY;
        double strafe = errorX * kP_brakeXY;
        double turn = errorH * kP_brakeH;

        forward = clip(forward, -1, 1);
        strafe = clip(strafe, -1, 1);
        turn = clip(turn, -1, 1);

        moveRobotFieldRelative(forward, strafe, turn);
    }

    public double getAlignTurnPower() {
        double goalHeading = 0;
        if (getPose().getY(DistanceUnit.INCH) >= 48 && alliance == DriveState.RED) {
            goalHeading = Math.atan2((yNearPoseRed - getPose().getY(DistanceUnit.INCH)),
                    xNearPoseRed - getPose().getX(DistanceUnit.INCH)) - Math.PI;
        }
        else if (getPose().getY(DistanceUnit.INCH) >= 48 && alliance == DriveState.BLUE) {
            goalHeading = Math.atan2((yNearPoseBlue - getPose().getY(DistanceUnit.INCH)),
                    xNearPoseBlue - getPose().getX(DistanceUnit.INCH)) - Math.PI;
        }
        else if (getPose().getY(DistanceUnit.INCH) < 48 && alliance == DriveState.RED) {
            goalHeading = Math.atan2((yFarPoseRed - getPose().getY(DistanceUnit.INCH)),
                    xFarPoseRed - getPose().getX(DistanceUnit.INCH)) - Math.PI;
        }
        else if (getPose().getY(DistanceUnit.INCH) < 48 && alliance == DriveState.BLUE) {
            goalHeading = Math.atan2((yFarPoseBlue - getPose().getY(DistanceUnit.INCH)),
                    xFarPoseBlue - getPose().getX(DistanceUnit.INCH)) - Math.PI;
        }
        double errorH = angleWrap(goalHeading - getPose().getHeading(DriveConstants.angleUnit));

        double turn = alignPID.calculate(getPose().getHeading(DriveConstants.angleUnit), goalHeading);

        return clip(turn, -1, 1);
    }

    public void visionCalibrate() {
        Pose3D visionPose = autoApriltag.getRobotPosition();
        if (visionPose != null) otos.setPosition(Util.visionPoseToOTOSPose(visionPose));
        yawOffset = alliance == DriveState.BLUE? Math.PI: 0;
    }

    public double distanceToGoal() {
        if (getPose().getY(DistanceUnit.INCH) >= 48 && alliance == DriveState.RED) {
            return poseDistance(getPose(), new Pose2D(DistanceUnit.INCH,
                    xNearPoseRed, yNearPoseRed, AngleUnit.RADIANS, 0));
        } else if (getPose().getY(DistanceUnit.INCH) >= 48 && alliance == DriveState.BLUE) {
            return poseDistance(getPose(), new Pose2D(DistanceUnit.INCH,
                    xNearPoseBlue, yNearPoseBlue, AngleUnit.RADIANS, 0));
        } else if (getPose().getY(DistanceUnit.INCH) < 48 && alliance == DriveState.RED) {
            return poseDistance(getPose(), new Pose2D(DistanceUnit.INCH,
                    xFarPoseRed, yFarPoseRed, AngleUnit.RADIANS, 0));
        } else if (getPose().getY(DistanceUnit.INCH) < 48 && alliance == DriveState.BLUE) {
            return poseDistance(getPose(), new Pose2D(DistanceUnit.INCH,
                    xFarPoseRed, yFarPoseRed, AngleUnit.RADIANS, 0));
        }
        return -1;
    }

    public double getShooterVelocity() {
        if (distanceToGoal() != -1) return ShooterConstants.slowVelocity
                + (ShooterConstants.fastVelocity - ShooterConstants.slowVelocity)
                / (farGoalDistance - nearGoalDistance) * (distanceToGoal() - nearGoalDistance);
        return 0;
    }

    @Override
    public void periodic() {
        if (driveState == DriveState.STOP) {
            applyBrake();
        }
//        else if (driveState == DriveState.ALIGN) {
//            moveRobotFieldRelative(0, 0, getAlignTurnPower());
//        }
        lastPose = getPose();
    }
}