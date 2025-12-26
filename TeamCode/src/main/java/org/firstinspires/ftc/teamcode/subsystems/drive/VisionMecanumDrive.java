package org.firstinspires.ftc.teamcode.subsystems.drive;

import static com.arcrobotics.ftclib.purepursuit.PurePursuitUtil.angleWrap;
import static com.qualcomm.robotcore.util.Range.clip;
import static org.firstinspires.ftc.teamcode.subsystems.drive.DriveConstants.farGoalDistance;
import static org.firstinspires.ftc.teamcode.subsystems.drive.DriveConstants.kD_alignH;
import static org.firstinspires.ftc.teamcode.subsystems.drive.DriveConstants.kF_alignH;
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
import static org.firstinspires.ftc.teamcode.subsystems.drive.DriveConstants.xPoseDW;
import static org.firstinspires.ftc.teamcode.subsystems.drive.DriveConstants.yFarPoseBlue;
import static org.firstinspires.ftc.teamcode.subsystems.drive.DriveConstants.yFarPoseRed;
import static org.firstinspires.ftc.teamcode.subsystems.drive.DriveConstants.yNearPoseBlue;
import static org.firstinspires.ftc.teamcode.subsystems.drive.DriveConstants.yNearPoseRed;
import static org.firstinspires.ftc.teamcode.subsystems.drive.DriveConstants.yPoseDW;
import static org.firstinspires.ftc.teamcode.utils.Util.adjustRange;
import static org.firstinspires.ftc.teamcode.utils.Util.poseDistance;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.subsystems.cds.CDS;
import org.firstinspires.ftc.teamcode.subsystems.shooter.ShooterConstants;
import org.firstinspires.ftc.teamcode.subsystems.vision.AutoApriltag;
import org.firstinspires.ftc.teamcode.utils.Util;

@Config
public class VisionMecanumDrive extends SubsystemBase {
    public final DcMotor leftFrontMotor, leftBackMotor, rightFrontMotor, rightBackMotor;

    private final GoBildaPinpointDriver od;
    private final AutoApriltag autoApriltag;
    private double yawOffset;// mm
    private PIDFController alignPID;
    private final DriveState alliance;
    private final CDS cds;

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

    public VisionMecanumDrive(final HardwareMap hardwareMap, DriveState alliance, CDS cds) {
        leftFrontMotor = hardwareMap.get(DcMotor.class, "leftFrontMotor");
        leftBackMotor = hardwareMap.get(DcMotor.class, "leftBackMotor");
        rightFrontMotor = hardwareMap.get(DcMotor.class, "rightFrontMotor");
        rightBackMotor = hardwareMap.get(DcMotor.class, "rightBackMotor");
        od = hardwareMap.get(GoBildaPinpointDriver.class, "od");
        driveState = DriveState.STOP;
        alignPID = new PIDFController(kP_alignH, kI_alignH, kD_alignH, kF_alignH);
        this.alliance = alliance;
        this.cds = cds;

        leftFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        autoApriltag = new AutoApriltag(hardwareMap);

        od.resetPosAndIMU();
        od.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.REVERSED);
        od.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        od.setOffsets(xPoseDW, yPoseDW, DriveConstants.distanceUnit);

        leftFrontMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBackMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        lastPose = new Pose2D(DriveConstants.distanceUnit, 0, 0, DriveConstants.angleUnit, 0);
    }

    public void stop() {
        moveRobot(0, 0, 0);
    }

    public void reset(double heading) {
        yawOffset = od.getPosition().getHeading(DriveConstants.angleUnit) + heading;
    }

    public void setDriveState(DriveState driveState) {
        this.driveState = driveState;
    }

    public void moveRobotFieldRelative(double forward, double fun, double turn) {

        double botHeading = od.getPosition().getHeading(DriveConstants.angleUnit) - yawOffset;
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
        return od.getPosition();
    }

    public double getYawOffset() {return yawOffset;}

    public boolean isHeadingAtSetPoint(double headingSetPoint) {
        return Util.epsilonEqual(od.getPosition().getHeading(DriveConstants.angleUnit), headingSetPoint,
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
        if (getPose().getY(DriveConstants.distanceUnit) >= 48 && alliance == DriveState.RED) {
            goalHeading = Math.atan2((yNearPoseRed - getPose().getY(DriveConstants.distanceUnit)),
                    xNearPoseRed - getPose().getX(DriveConstants.distanceUnit)) - Math.PI;
        }
        else if (getPose().getY(DriveConstants.distanceUnit) >= 48 && alliance == DriveState.BLUE) {
            goalHeading = Math.atan2((yNearPoseBlue - getPose().getY(DriveConstants.distanceUnit)),
                    xNearPoseBlue - getPose().getX(DriveConstants.distanceUnit)) - Math.PI;
        }
        else if (getPose().getY(DriveConstants.distanceUnit) < 48 && alliance == DriveState.RED) {
            goalHeading = Math.atan2((yFarPoseRed - getPose().getY(DriveConstants.distanceUnit)),
                    xFarPoseRed - getPose().getX(DriveConstants.distanceUnit)) - Math.PI;
        }
        else if (getPose().getY(DriveConstants.distanceUnit) < 48 && alliance == DriveState.BLUE) {
            goalHeading = Math.atan2((yFarPoseBlue - getPose().getY(DriveConstants.distanceUnit)),
                    xFarPoseBlue - getPose().getX(DriveConstants.distanceUnit)) - Math.PI;
        }

        double turn = alignPID.calculate(0,
                adjustRange(goalHeading - getPose().getHeading(DriveConstants.angleUnit)));

        return clip(turn, -1, 1);
    }

    public Pose3D getVisionPose() {
        return autoApriltag.getRobotPosition();
    }

    public void visionCalibrate() {
        Pose3D visionPose = autoApriltag.getRobotPosition();
        if (visionPose != null){
            od.setPosition(Util.visionPoseToDWPose(visionPose));
            od.setHeading(Util.visionPoseToDWPose(visionPose).getHeading(DriveConstants.angleUnit),
                    DriveConstants.angleUnit);
            cds.setLED(CDS.LEDState.BLUE, 1000);
        }
        yawOffset = alliance == DriveState.BLUE? Math.PI: 0;
    }

    public double distanceToGoal() {
        if (getPose().getY(DriveConstants.distanceUnit) >= 48 && alliance == DriveState.RED) {
            return poseDistance(getPose(), new Pose2D(DriveConstants.distanceUnit,
                    xNearPoseRed, yNearPoseRed, DriveConstants.angleUnit, 0));
        } else if (getPose().getY(DriveConstants.distanceUnit) >= 48 && alliance == DriveState.BLUE) {
            return poseDistance(getPose(), new Pose2D(DriveConstants.distanceUnit,
                    xNearPoseBlue, yNearPoseBlue, DriveConstants.angleUnit, 0));
        } else if (getPose().getY(DriveConstants.distanceUnit) < 48 && alliance == DriveState.RED) {
            return poseDistance(getPose(), new Pose2D(DriveConstants.distanceUnit,
                    xFarPoseRed, yFarPoseRed, DriveConstants.angleUnit, 0));
        } else if (getPose().getY(DriveConstants.distanceUnit) < 48 && alliance == DriveState.BLUE) {
            return poseDistance(getPose(), new Pose2D(DriveConstants.distanceUnit,
                    xFarPoseRed, yFarPoseRed, DriveConstants.angleUnit, 0));
        }
        return -1;
    }

    public double getShooterVelocity() {
        if (distanceToGoal() != -1) return 20 * Math.ceil((ShooterConstants.slowVelocity
                + (ShooterConstants.fastVelocity - ShooterConstants.slowVelocity)
                / (farGoalDistance - nearGoalDistance) * (distanceToGoal() - nearGoalDistance)) / 20);
        return 0;
    }

    public double getShooterPower() {
        if (distanceToGoal() != -1) return ShooterConstants.slowPower
                + (ShooterConstants.fastPower - ShooterConstants.slowPower)
                / (farGoalDistance - nearGoalDistance) * (distanceToGoal() - nearGoalDistance);
        return 0;
    }

    @Override
    public void periodic() {
        od.update();
        if (driveState == DriveState.STOP) {
            applyBrake();
        }
//        else if (driveState == DriveState.ALIGN) {
//            moveRobotFieldRelative(0, 0, getAlignTurnPower());
//        }
        lastPose = getPose();
    }
}