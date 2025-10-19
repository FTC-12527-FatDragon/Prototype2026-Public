package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

class PID {
    public double kP, kI, kD;

    PID(int kP, int kI, int kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
    }
}

class Filter {
    final int WINDOW = 10;
    double[] history = new double[WINDOW];
    public int index = 0;

    public void updateVelocity(double newVal) {
        history[index] = newVal;
        index = (index + 1) % WINDOW;
    }

    public double getSmoothedVelocity() {
        double sum = 0;
        for (double v : history) sum += v;
        return sum / WINDOW;
    }
}

@TeleOp(name = "MotorTunerUltimate")
@Config
public class MotorTunerUltimate extends LinearOpMode {
    public static String[] motorName = {"", "", "", ""};
    public static double[] motorPower = new double[4];
    public static boolean[] closeLoop = new boolean[4];
    public static double[] target = new double[4];
    public static boolean[] isVelocityCloseLoop = new boolean[4];
    public double[] lastPos = new double[4];

    public static PID[] PIDs = {
            new PID(0, 0, 0),
            new PID(0, 0, 0),
            new PID(0, 0, 0),
            new PID(0, 0, 0)
    };

    DcMotorEx[] motors = new DcMotorEx[4];

    PIDController[] pidControllers = {
            new PIDController(0, 0, 0),
            new PIDController(0, 0, 0),
            new PIDController(0, 0, 0),
            new PIDController(0, 0, 0)
    };

    public double getVelocity(int index, double pos) {
        return (pos - lastPos[index]) / 0.02 * 60;
    }

    Filter filter = new Filter();

    @Override
    public void runOpMode() {
        FtcDashboard dashboard = FtcDashboard.getInstance();

        for (int i = 0; i < 4; ++i)
            if (!motorName[i].isEmpty()) {
                motors[i] = hardwareMap.get(DcMotorEx.class, motorName[i]);
                if (closeLoop[i]) {
                    motors[i].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    pidControllers[i].setPID(PIDs[i].kP, PIDs[i].kI, PIDs[i].kD);
                }
            }

        waitForStart();

        while (opModeIsActive()) {
            for (int i = 0; i < 4; ++i)
                if (!motorName[i].isEmpty()) {
                    if (closeLoop[i] && isVelocityCloseLoop[i]) {
                        if (filter.index == 0) {
                            pidControllers[i].setPID(PIDs[i].kP, PIDs[i].kI, PIDs[i].kD);

                            double v = filter.getSmoothedVelocity();
                            motors[i].setPower(pidControllers[i].calculate(v, target[i]));

                            TelemetryPacket packet = new TelemetryPacket();
                            packet.put("targetVelocity " + i, target[i]);
                            packet.put("currentVelocity " + i, v);

                            dashboard.sendTelemetryPacket(packet);
                        }
                        double pos = motors[i].getCurrentPosition();
                        filter.updateVelocity(getVelocity(i, pos));
                        lastPos[i] = pos;
                    }
                    else
                        motors[i].setPower(motorPower[i]);
                }
        }
    }
}
