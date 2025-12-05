package org.firstinspires.ftc.teamcode.subsystems.cds;

import static org.firstinspires.ftc.teamcode.subsystems.cds.CDSConstants.SCALE_FACTOR;
import static org.firstinspires.ftc.teamcode.subsystems.cds.CDSConstants.ballDistance;
import static org.firstinspires.ftc.teamcode.subsystems.cds.CDSConstants.purpleConst;

import android.graphics.Color;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.subsystems.vision.VisionConstants;

import java.util.ArrayList;
import java.util.Collections;
import java.util.LinkedList;
import java.util.List;
import java.util.Queue;

public class CDS extends SubsystemBase {
    private final ColorSensor colorSensor;
    private final DistanceSensor distanceSensor;
    private final Servo led;

    private final float[] hsvValues = {0F, 0F, 0F};

    public CDS(HardwareMap hardwareMap) {
        colorSensor = hardwareMap.get(ColorSensor.class, CDSConstants.colorSensorName);
        distanceSensor = hardwareMap.get(DistanceSensor.class, CDSConstants.distanceSensorName);
        led = hardwareMap.get(Servo.class, CDSConstants.ledName);
    }

    private boolean ballDetected = false;
    private boolean purpule = false;
    private boolean green = false;

    private long ballNum = 0;

    private Queue<Integer> colorQue = new LinkedList<>();

    private List<Float> hues = new ArrayList<>();

    public void deleteFirst() {
        colorQue.poll();
    }

    public int getFirst() {
        if (colorQue.peek() != null) return colorQue.peek();
        return -1;
    }

    public String getColorQue() {
        return colorQue.toString();
    }

    public long getBallNum() {
        return ballNum;
    }

    public void deleteBalls() {
        ballNum = 0;
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

        if (dis < ballDistance) {
            hues.add(hsvValues[0]);
            if (!ballDetected) ballNum++;
            ballDetected = true;
        }

        if (dis > ballDistance && ballDetected) {
            ballDetected = false;

            Collections.sort(hues);

            float res = hues.get(hues.size() / 2);

            if (res >= purpleConst) {
                colorQue.offer(1);
            }
            else {
                colorQue.offer(0);
            }
            if (colorQue.size() > 3) colorQue.poll();

            purpule = false;
            green = false;

            hues.clear();
        }
        if (ballNum >= 3) led.setPosition(1);
        else led.setPosition(0);
    }
}
