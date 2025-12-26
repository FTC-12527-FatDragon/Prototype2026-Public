package org.firstinspires.ftc.teamcode.subsystems.cds;

import static org.firstinspires.ftc.teamcode.subsystems.cds.CDSConstants.ballDistance;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class CDS extends SubsystemBase {
    private final ColorSensor colorSensor;
    private final DistanceSensor distanceSensor;
    private final ColorSensor colorSensor1;
    private final DistanceSensor distanceSensor1;
    private final Servo led;
    private final ElapsedTime ledTimer;
    public final TelemetryPacket packet = new TelemetryPacket();

    private boolean killed;
    private boolean isRainbow;
    private double dis, dis1, ledTime;
    private double r, g, b;

    private final float[] hsvValues = {0F, 0F, 0F};

    public enum LEDState {
        NULL(0.2),
        RED(0.3),
        YELLOW(0.4),
        GREEN(0.5),
        BLUE(0.6),
        PURPLE(0.7),
        WHITE(0.8);

        public double colorPose;

        LEDState(double colorPose) {
            this.colorPose = colorPose;
        }
    }

    public CDS(HardwareMap hardwareMap) {
        colorSensor = hardwareMap.get(ColorSensor.class, CDSConstants.colorSensorName);
        distanceSensor = hardwareMap.get(DistanceSensor.class, CDSConstants.colorSensorName);
        colorSensor1 = hardwareMap.get(ColorSensor.class, CDSConstants.colorSensor1Name);
        distanceSensor1 = hardwareMap.get(DistanceSensor.class, CDSConstants.colorSensor1Name);
        led = hardwareMap.get(Servo.class, CDSConstants.ledName);
        ledTimer = new ElapsedTime();
        this.killed = false;
        this.isRainbow = false;
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

    public void setLED(LEDState color) {
        led.setPosition(color.colorPose);
    }

    public void setLED(LEDState color, double time) {
        led.setPosition(color.colorPose);
        ledTimer.reset();
        ledTime = time;
    }

    public void startRainbow() {
        isRainbow = true;
    }

    public void kill() {
        killed = true;
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

        if (killed) {
            ballNum = 0;
        }


        if(ledTimer.milliseconds() > ledTime) {
            if (killed) setLED(LEDState.WHITE);
            else if (isRainbow) {
                led.setPosition(ledTimer.milliseconds() % 2000 / 5000 + 0.3);
            }
            else {
                switch ((int) ballNum) {
                    case 0:
                        setLED(LEDState.PURPLE);
                        break;
                    case 1:
                        setLED(LEDState.RED);
                        break;
                    case 2:
                        setLED(LEDState.YELLOW);
                        break;
                    case 3:
                        setLED(LEDState.GREEN);
                        break;
                }
            }
        }

        packet.put("colorSensor dis", dis);
        packet.put("colorSensor dis1", dis1);
        FtcDashboard.getInstance().sendTelemetryPacket(packet);
    }
}
