package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.utils.DcMotorRe;

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

    public static boolean[] isVelocityCloseLoop = new boolean[4];

    public static PID[] PIDs = {
            new PID(0, 0, 0),
            new PID(0, 0, 0),
            new PID(0, 0, 0),
            new PID(0, 0, 0)
    };

    DcMotorRe[] motors = new DcMotorRe[4];

    Servo[] servos = new Servo[4];

    CRServo[] crServos = new CRServo[4];

    PIDController[] pidControllers = {
            new PIDController(0, 0, 0),
            new PIDController(0, 0, 0),
            new PIDController(0, 0, 0),
            new PIDController(0, 0, 0)
    };

    @Override
    public void runOpMode() {
        FtcDashboard dashboard = FtcDashboard.getInstance();

        for (int i = 0; i < 4; ++i)
            if (!motorName[i].isEmpty()) {
                motors[i] = new DcMotorRe(hardwareMap, motorName[i]);
                pidControllers[i].setPID(PIDs[i].kP, PIDs[i].kI, PIDs[i].kD);
            }

        waitForStart();

        while (opModeIsActive()) {
            for (int i = 0; i < 4; ++i) {
                if (!motorName[i].isEmpty()) {
                    if (closeLoop[i] && isVelocityCloseLoop[i]) {
                        pidControllers[i].setPID(PIDs[i].kP, PIDs[i].kI, PIDs[i].kD);

                        double v = motors[i].getAverageVelocity();
                        motors[i].setPower(pidControllers[i].calculate(v, motorTarget[i]));

                        TelemetryPacket packet = new TelemetryPacket();
                        packet.put("targetVelocity " + i, motorTarget[i]);
                        packet.put("currentVelocity " + i, v);

                        dashboard.sendTelemetryPacket(packet);
                    }
                    if (!closeLoop[i]) {
                        motors[i].setPower(motorTarget[i]);
                        double v = motors[i].getAverageVelocity();

                        TelemetryPacket packet = new TelemetryPacket();
                        packet.put("currentVelocity " + i, v);
                        packet.put("LibVelocity " + i, motors[i].getLibVelocity());

                        dashboard.sendTelemetryPacket(packet);
                    }
                    motors[i].updateLastPos();
                }

                if (!servoName[i].isEmpty()) {

                }
            }
        }
    }
}
