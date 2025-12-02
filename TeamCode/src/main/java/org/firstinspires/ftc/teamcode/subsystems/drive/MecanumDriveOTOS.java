package org.firstinspires.ftc.teamcode.subsystems.drive;

import static com.arcrobotics.ftclib.purepursuit.PurePursuitUtil.angleWrap;
import static com.qualcomm.robotcore.util.Range.clip;
import static org.firstinspires.ftc.teamcode.subsystems.drive.DriveConstants.kP_h;
import static org.firstinspires.ftc.teamcode.subsystems.drive.DriveConstants.kP_xy;
import static org.firstinspires.ftc.teamcode.subsystems.drive.DriveConstants.strafingBalance;
import static org.firstinspires.ftc.teamcode.subsystems.drive.DriveConstants.xFarPoseBlue;
import static org.firstinspires.ftc.teamcode.subsystems.drive.DriveConstants.xFarPoseRed;
import static org.firstinspires.ftc.teamcode.subsystems.drive.DriveConstants.xPoseBlue;
import static org.firstinspires.ftc.teamcode.subsystems.drive.DriveConstants.xPoseRed;
import static org.firstinspires.ftc.teamcode.subsystems.drive.DriveConstants.yFarPoseBlue;
import static org.firstinspires.ftc.teamcode.subsystems.drive.DriveConstants.yFarPoseRed;
import static org.firstinspires.ftc.teamcode.subsystems.drive.DriveConstants.yPoseBlue;
import static org.firstinspires.ftc.teamcode.subsystems.drive.DriveConstants.yPoseRed;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

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
    private DriveState alliance;

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

    private void applyBreak() {
        Pose2D p = getPose();

        double errorX = lastPose.getX(DriveConstants.distanceUnit) - p.getX(DriveConstants.distanceUnit);
        double errorY = lastPose.getY(DriveConstants.distanceUnit) - p.getY(DriveConstants.distanceUnit);
        double errorH = angleWrap(lastPose.getHeading(DriveConstants.angleUnit) - p.getHeading(DriveConstants.angleUnit));

        double forward =  errorY * kP_xy;
        double strafe  =  errorX * kP_xy;
        double turn    =  errorH * kP_h;
        
        forward = clip(forward, -1, 1);
        strafe  = clip(strafe,  -1, 1);
        turn    = clip(turn,    -1, 1);

        moveRobotFieldRelative(forward, strafe, turn);
    }

    /**
     * @param heading in radians
     */
    private void alignTo(double heading) {
        double errorH = angleWrap(heading - lastPose.getHeading(DriveConstants.angleUnit));

        double turn    =  errorH * kP_h;

        turn    = clip(turn,    -1, 1);

        moveRobotFieldRelative(0, 0, turn);
    }

    public void visionCalibrate() {
        Pose3D visionPose = autoApriltag.getRobotPosition();
        if (visionPose != null) otos.setPosition(Util.visionPoseToOTOSPose(visionPose, alliance));
    }

    @Override
    public void periodic() {
        if (driveState == DriveState.STOP) {
            applyBreak();
        }
        else if (driveState == DriveState.ALIGN) {
            if (getPose().getY(DistanceUnit.INCH) >= 48 && alliance == DriveState.RED) {
                double goalHeading = Math.atan2((yPoseRed - getPose().getY(DistanceUnit.INCH)),
                        xPoseRed - getPose().getX(DistanceUnit.INCH)) - Math.PI;
                alignTo(goalHeading);
            }
            else if (getPose().getY(DistanceUnit.INCH) >= 48 && alliance == DriveState.BLUE) {
                double goalHeading = Math.atan2((yPoseBlue - getPose().getY(DistanceUnit.INCH)),
                        xPoseBlue - getPose().getX(DistanceUnit.INCH)) - Math.PI;
                alignTo(goalHeading);
            }
            else if (getPose().getY(DistanceUnit.INCH) < 48 && alliance == DriveState.RED) {
                double goalHeading = Math.atan2((yFarPoseRed - getPose().getY(DistanceUnit.INCH)),
                        xFarPoseRed - getPose().getX(DistanceUnit.INCH)) - Math.PI;
                alignTo(goalHeading);
            }
            else if (getPose().getY(DistanceUnit.INCH) < 48 && alliance == DriveState.BLUE) {
                double goalHeading = Math.atan2((yFarPoseBlue - getPose().getY(DistanceUnit.INCH)),
                        xFarPoseBlue - getPose().getX(DistanceUnit.INCH)) - Math.PI;
                alignTo(goalHeading);
            }
        }
        lastPose = getPose();
    }
}