package org.firstinspires.ftc.teamcode.tests;

import android.graphics.Color;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.ArrayList;
import java.util.Collections;
import java.util.LinkedList;
import java.util.List;
import java.util.Locale;
import java.util.Queue;

class PID {
    public double kP, kI, kD;

    PID(int kP, int kI, int kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
    }
}

@TeleOp(name = "DashTuner")
@Config
public class DashTuner extends LinearOpMode {
    public static String[] motorName = {"", "", "", ""};
    public static String[] servoName = {"", "", "", ""};
    public static boolean[] closeLoop = new boolean[4];
    public static double[] motorTarget = new double[4];
    public static double[] servoTarget = new double[4];
    public static double[] slaveTo = {-1, -1, -1, -1};

    public static boolean[] isVelocityCloseLoop = new boolean[4];

    public static PID[] PIDs = {
            new PID(0, 0, 0),
            new PID(0, 0, 0),
            new PID(0, 0, 0),
            new PID(0, 0, 0)
    };

    DcMotorEx[] motors = new DcMotorEx[4];

    Servo[] servos = new Servo[4];

    PIDController[] pidControllers = {
            new PIDController(0, 0, 0),
            new PIDController(0, 0, 0),
            new PIDController(0, 0, 0),
            new PIDController(0, 0, 0)
    };

    public static String colorSensorName = "";

    ColorSensor colorSensor;

    DistanceSensor distanceSensor;

    float hsvValues[] = {0F, 0F, 0F};

    final float values[] = hsvValues;

    final double SCALE_FACTOR = 255;

    private boolean ballDetected = false;
    private boolean purpule = false;
    private boolean green = false;

    private double lastR = 0.0, lastG = 0.0, lastB = 0.0;

    private Queue<Integer> colorQue = new LinkedList<>();

    List<Float> hues = new ArrayList<>();

    @Override
    public void runOpMode() {
        FtcDashboard dashboard = FtcDashboard.getInstance();

        for (int i = 0; i < 4; i++) {
            if (!motorName[i].isEmpty()) {
                motors[i] = hardwareMap.get(DcMotorEx.class, motorName[i]);
                pidControllers[i].setPID(PIDs[i].kP, PIDs[i].kI, PIDs[i].kD);
            }
            if (!servoName[i].isEmpty()) {
                servos[i] = hardwareMap.get(Servo.class, servoName[i]);
            }
        }

        if (!colorSensorName.isEmpty()) {
            colorSensor = hardwareMap.get(ColorSensor.class, colorSensorName);
            distanceSensor = hardwareMap.get(DistanceSensor.class, colorSensorName);
        }

        waitForStart();

        while (opModeIsActive()) {
            for (int i = 0; i < 4; i++) {
                if (!motorName[i].isEmpty()) {
                    if (slaveTo[i] != -1) {
                        if (closeLoop[i])
                            motors[i].setPower(-pidControllers[(int) slaveTo[i]].calculate(motors[(int)
                                    slaveTo[i]].getVelocity(), motorTarget[(int) slaveTo[i]]));
                        else
                            motors[i].setPower(-motorTarget[(int) slaveTo[i]]);
                    }
                    else if (closeLoop[i] && isVelocityCloseLoop[i]) {
                        pidControllers[i].setPID(PIDs[i].kP, PIDs[i].kI, PIDs[i].kD);

                        double v = motors[i].getVelocity();

                        motors[i].setPower(pidControllers[i].calculate(v, motorTarget[i]));

                        TelemetryPacket packet = new TelemetryPacket();
                        packet.put("targetVelocity " + i, motorTarget[i]);
                        packet.put("Velocity " + i, v);

                        dashboard.sendTelemetryPacket(packet);
                    }
                    else if (closeLoop[i] && !isVelocityCloseLoop[i]) {
                        pidControllers[i].setPID(PIDs[i].kP, PIDs[i].kI, PIDs[i].kD);

                        double pos = motors[i].getCurrentPosition();

                        motors[i].setPower(pidControllers[i].calculate(pos, motorTarget[i]));

                        TelemetryPacket packet = new TelemetryPacket();
                        packet.put("targetPosition " + i, motorTarget[i]);
                        packet.put("currentPosition " + i, pos);

                        dashboard.sendTelemetryPacket(packet);
                    }
                    else if (!closeLoop[i]) {
                        motors[i].setPower(motorTarget[i]);
                        double v = motors[i].getVelocity();

                        TelemetryPacket packet = new TelemetryPacket();
                        packet.put("currentVelocity " + i, v);
                        packet.put("Velocity " + i, motors[i].getVelocity());

                        dashboard.sendTelemetryPacket(packet);
                    }
                }

                if (!servoName[i].isEmpty()) {
                    servos[i].setPosition(servoTarget[i]);
                }

                if (!colorSensorName.isEmpty()) {
                    Color.RGBToHSV((int) (colorSensor.red() * SCALE_FACTOR),
                            (int) (colorSensor.green() * SCALE_FACTOR),
                            (int) (colorSensor.blue() * SCALE_FACTOR),
                            hsvValues);

                    TelemetryPacket packet = new TelemetryPacket();

                    packet.put("Alpha", colorSensor.alpha());
                    packet.put("Red  ", colorSensor.red());
                    packet.put("Green", colorSensor.green());
                    packet.put("Blue ", colorSensor.blue());
                    packet.put("Hue", hsvValues[0]);
                    packet.put("Distance (cm)", String.format(Locale.US, "%.02f", distanceSensor.getDistance(DistanceUnit.CM)));

                    double dis = distanceSensor.getDistance(DistanceUnit.CM);

                    double r = colorSensor.red();
                    double g = colorSensor.green();
                    double b = colorSensor.blue();

                    Color.RGBToHSV((int) (r * SCALE_FACTOR),
                            (int) (g * SCALE_FACTOR),
                            (int) (b * SCALE_FACTOR),
                            hsvValues);

                    if (dis < 4.0) {
                        hues.add(hsvValues[0]);
                        ballDetected = true;
                    }

                    if (dis > 4.0 && ballDetected) {
                        ballDetected = false;

                        Collections.sort(hues);

                        float res = hues.get(hues.size() / 2);

                        if (res >= 180) {
                            colorQue.offer(1);
                        }
                        else {
                            colorQue.offer(0);
                        }

                        purpule = false;
                        green = false;

                        hues.clear();
                    }

                    packet.put("BALL COLORS", colorQue);

                    dashboard.sendTelemetryPacket(packet);

                    lastG = g; lastB = b; lastR = r;
                }
            }
        }
    }
}
