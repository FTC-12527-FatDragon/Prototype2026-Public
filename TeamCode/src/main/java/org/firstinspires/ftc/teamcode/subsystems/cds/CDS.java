package org.firstinspires.ftc.teamcode.subsystems.cds;

import static org.firstinspires.ftc.teamcode.subsystems.cds.CDSConstants.SCALE_FACTOR;
import static org.firstinspires.ftc.teamcode.subsystems.cds.CDSConstants.ballDistance;
import static org.firstinspires.ftc.teamcode.subsystems.cds.CDSConstants.purpleConst;

import android.graphics.Color;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
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
    private final ColorSensor colorSensor1;
    private final DistanceSensor distanceSensor1;
    private final Servo led;
    public final TelemetryPacket packet = new TelemetryPacket();

    private double dis, dis1;
    private double r, g, b;

    private final float[] hsvValues = {0F, 0F, 0F};

    public CDS(HardwareMap hardwareMap) {
        colorSensor = hardwareMap.get(ColorSensor.class, CDSConstants.colorSensorName);
        distanceSensor = hardwareMap.get(DistanceSensor.class, CDSConstants.colorSensorName);
        colorSensor1 = hardwareMap.get(ColorSensor.class, CDSConstants.colorSensor1Name);
        distanceSensor1 = hardwareMap.get(DistanceSensor.class, CDSConstants.colorSensor1Name);
        led = hardwareMap.get(Servo.class, CDSConstants.ledName);
    }

    private boolean ballDetected = false;

    private long ballNum = 0;

//    private Queue<Integer> colorQue = new LinkedList<>();
//
//    private List<Float> hues = new ArrayList<>();
//
//    public void deleteFirst() {
//        colorQue.poll();
//    }
//
//    public int getFirst() {
//        if (colorQue.peek() != null) return colorQue.peek();
//        return -1;
//    }
//
//    public String getColorQue() {
//        return colorQue.toString();
//    }

    public long getBallNum() {
        return ballNum;
    }

    public void deleteBalls() {
        ballNum = 0;
    }

    @Override
    public void periodic() {
        dis = distanceSensor.getDistance(DistanceUnit.CM);
        dis1 = distanceSensor1.getDistance(DistanceUnit.CM);

//        r = colorSensor.red();
//        g = colorSensor.green();
//        b = colorSensor.blue();
//
//        Color.RGBToHSV((int) (r * SCALE_FACTOR),
//                (int) (g * SCALE_FACTOR),
//                (int) (b * SCALE_FACTOR),
//                hsvValues);

        if (dis < ballDistance || dis1 < ballDistance) {
//            hues.add(hsvValues[0]);
            if (!ballDetected) ballNum++;
            ballDetected = true;
        }

        if ((dis > ballDistance && dis1 > ballDistance) && ballDetected) {
            ballDetected = false;

//            Collections.sort(hues);
//            float res = hues.get(hues.size() / 2);
//            if (res >= purpleConst) {
//                colorQue.offer(1);
//            }
//            else {
//                colorQue.offer(0);
//            }
//            if (colorQue.size() > 3) colorQue.poll();
//            hues.clear();
        }
        if (ballNum >= 3) led.setPosition(1);
        else led.setPosition(0);

        packet.put("colorSensor dis", dis);
        packet.put("colorSensor dis1", dis1);
        FtcDashboard.getInstance().sendTelemetryPacket(packet);
    }
}
