package org.firstinspires.ftc.teamcode.subsystems.drive;

import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.OTOSConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(5)
            .forwardZeroPowerAcceleration(-26.982707557642915)
            .lateralZeroPowerAcceleration(-67.19294908602483)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.15, 0, 0.01, 0.025))
            .headingPIDFCoefficients(new PIDFCoefficients(2, 0, 0.05, 0.025))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.03,0.0,0.000005,0,0.0))
            .centripetalScaling(0);


    public static MecanumConstants driveConstants = new MecanumConstants()
            .leftFrontMotorName("leftFrontMotor")
            .leftRearMotorName("leftBackMotor")
            .rightFrontMotorName("rightFrontMotor")
            .rightRearMotorName("rightBackMotor")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .xVelocity(80.89569139856053)
            .yVelocity(63.59879188837967)
            .maxPower(1);

    public static OTOSConstants otosConstants = new OTOSConstants()
            .hardwareMapName("otos")
            .linearUnit(DistanceUnit.MM)
            .angleUnit(AngleUnit.RADIANS)
            .offset(new SparkFunOTOS.Pose2D(
                    DriveConstants.xPoseOTOS,
                    DriveConstants.yPoseOTOS,
                    DriveConstants.headingPoseOTOS
            ));

//    public static PinpointConstants localizerConstants = new PinpointConstants()
//            .forwardPodY(0)
//            .strafePodX(6.5)
//            .distanceUnit(DistanceUnit.INCH)
//            .hardwareMapName("od")
//            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
//            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED)
//            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED);
    /**
     These are the PathConstraints in order:
     tValueConstraint, velocityConstraint, translationalConstraint, headingConstraint, timeoutConstraint,
     brakingStrength, BEZIER_CURVE_SEARCH_LIMIT, brakingStart

     The BEZIER_CURVE_SEARCH_LIMIT should typically be left at 10 and shouldn't be changed.
     */

//    public static PathConstraints pathConstraints = new PathConstraints(
//            0.995,
//            0.1,
//            0.1,
//            0.009,
//            50,
//            1.25,
//            10,
//            1
//    );

    //Add custom localizers or drivetrains here
    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .mecanumDrivetrain(driveConstants)
                .OTOSLocalizer(otosConstants)
//                .pinpointLocalizer(localizerConstants)
//                .pathConstraints(pathConstraints)
                .build();
    }
}