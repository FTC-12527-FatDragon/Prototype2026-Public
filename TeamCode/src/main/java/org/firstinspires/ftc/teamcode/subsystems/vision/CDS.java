package org.firstinspires.ftc.teamcode.subsystems.vision;

import static org.firstinspires.ftc.teamcode.subsystems.vision.VisionConstants.SCALE_FACTOR;
import static org.firstinspires.ftc.teamcode.subsystems.vision.VisionConstants.purpleConst;

import android.graphics.Color;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class CDS extends SubsystemBase {
    private final ColorSensor colorSensor;
    private final DistanceSensor distanceSensor;

    private final float[] hsvValues = {0F, 0F, 0F};

    private boolean purpleDetected, greenDetected, ballDetected;

    public CDS(HardwareMap hardwareMap) {
        colorSensor = hardwareMap.get(ColorSensor.class, VisionConstants.colorSensorName);
        distanceSensor = hardwareMap.get(DistanceSensor.class, VisionConstants.distanceSensorName);

        purpleDetected = false;
        greenDetected = false;
        ballDetected = false;
    }

    public boolean ballIn() {
        return ballDetected;
    }

    public void

    @Override
    public void periodic() {
        double dis = distanceSensor.getDistance(DistanceUnit.CM);

        double r = colorSensor.red();
        double g = colorSensor.green();
        double b = colorSensor.blue();

        Color.RGBToHSV((int) (r * SCALE_FACTOR),
                (int) (g * SCALE_FACTOR),
                (int) (b * SCALE_FACTOR),
                hsvValues);
        if (dis < VisionConstants.ballDistance) {
            ballDetected = false;
        }

        purpleDetected = ballDetected && b > Math.max(r, g) * purpleConst;
    }
}
