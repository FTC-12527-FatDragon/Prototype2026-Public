package org.firstinspires.ftc.teamcode.utils;

import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.opencv.core.CvType;
import org.opencv.core.Mat;

public class Util {
    public static Pose Pose2DToPose(Pose2D pose2D) {
        return new Pose(pose2D.getX(DistanceUnit.INCH),
                pose2D.getY(DistanceUnit.INCH),
                pose2D.getHeading(AngleUnit.RADIANS));
    }

    public static boolean epsilonEqual(double a, double b, double epsilon) {
        return Math.abs(a - b) <= epsilon;
    }

    public static Pose3D compose(Pose3D A, Pose3D B) {
        double ax = A.getPosition().x;
        double ay = A.getPosition().y;
        double az = A.getPosition().z;

        double bx = B.getPosition().x;
        double by = B.getPosition().y;
        double bz = B.getPosition().z;

        double aRoll  = A.getOrientation().getRoll();
        double aPitch = A.getOrientation().getPitch();
        double aYaw   = A.getOrientation().getYaw();

        double bRoll  = B.getOrientation().getRoll();
        double bPitch = B.getOrientation().getPitch();
        double bYaw   = B.getOrientation().getYaw();

        double[][] RA = rpyToMatrix(aRoll, aPitch, aYaw);
        double[][] RB = rpyToMatrix(bRoll, bPitch, bYaw);

        double[][] R = multiply(RA, RB);

        // Actually we need: posInField = posA + RA * posB
        double[] posB = new double[]{bx, by, bz};
        double[] rotatedB = multMatVec(RA, posB);

        double rx = ax + rotatedB[0];
        double ry = ay + rotatedB[1];
        double rz = az + rotatedB[2];

        double[] rpy = matrixToRpy(R);
        double rRoll  = rpy[0];
        double rPitch = rpy[1];
        double rYaw   = rpy[2];

        return new Pose3D(new Position(DistanceUnit.MM, rx, ry, rz, A.getPosition().acquisitionTime),
                new YawPitchRollAngles(AngleUnit.RADIANS, rRoll, rPitch, rYaw, A.getOrientation().getAcquisitionTime()));
    }

    private static double[][] rpyToMatrix(double roll, double pitch, double yaw) {
        double sr = Math.sin(roll),  cr = Math.cos(roll);
        double sp = Math.sin(pitch), cp = Math.cos(pitch);
        double sy = Math.sin(yaw),   cy = Math.cos(yaw);

        // R = Rz(yaw) * Ry(pitch) * Rx(roll)
        double[][] R = new double[3][3];

        R[0][0] = cy * cp;
        R[0][1] = cy * sp * sr - sy * cr;
        R[0][2] = cy * sp * cr + sy * sr;

        R[1][0] = sy * cp;
        R[1][1] = sy * sp * sr + cy * cr;
        R[1][2] = sy * sp * cr - cy * sr;

        R[2][0] = -sp;
        R[2][1] = cp * sr;
        R[2][2] = cp * cr;

        return R;
    }

    private static double[] multMatVec(double[][] M, double[] v) {
        return new double[]{
                M[0][0]*v[0] + M[0][1]*v[1] + M[0][2]*v[2],
                M[1][0]*v[0] + M[1][1]*v[1] + M[1][2]*v[2],
                M[2][0]*v[0] + M[2][1]*v[1] + M[2][2]*v[2]
        };
    }

    private static double[][] multiply(double[][] A, double[][] B) {
        double[][] C = new double[3][3];
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                C[i][j] = 0;
                for (int k = 0; k < 3; ++k) {
                    C[i][j] += A[i][k] * B[k][j];
                }
            }
        }
        return C;
    }

    private static double[] matrixToRpy(double[][] R) {
        double sy = Math.sqrt(R[0][0]*R[0][0] + R[1][0]*R[1][0]);
        double roll, pitch, yaw;

        if (sy > 1e-9) {
            pitch = Math.atan2(-R[2][0], sy);
            yaw   = Math.atan2(R[1][0], R[0][0]);
            roll  = Math.atan2(R[2][1], R[2][2]);
        } else {
            pitch = Math.atan2(-R[2][0], sy); // sy ~ 0
            yaw   = Math.atan2(-R[0][1], R[1][1]);
            roll  = 0.0;
        }
        return new double[]{roll, pitch, yaw};
    }
}
