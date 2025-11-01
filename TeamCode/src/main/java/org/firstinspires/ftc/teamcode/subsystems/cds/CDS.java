package org.firstinspires.ftc.teamcode.subsystems.cds;

import static org.firstinspires.ftc.teamcode.subsystems.cds.CDSConstants.SCALE_FACTOR;
import static org.firstinspires.ftc.teamcode.subsystems.cds.CDSConstants.purpleConst;

import android.graphics.Color;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.subsystems.vision.VisionConstants;

public class CDS extends SubsystemBase {
    private final ColorSensor colorSensor;
    private final DistanceSensor distanceSensor;

    private final float[] hsvValues = {0F, 0F, 0F};

    private boolean purpleDetected, greenDetected, ballDetected;

    private double lastPos = 10.0;

    public CDS(HardwareMap hardwareMap) {
        colorSensor = hardwareMap.get(ColorSensor.class, CDSConstants.colorSensorName);
        distanceSensor = hardwareMap.get(DistanceSensor.class, CDSConstants.distanceSensorName);
    }



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

        purpleDetected = ballDetected && b > Math.max(r, g) * purpleConst;
    }
}
