package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "MotorsRun")
@Config
public class MotorsRun extends LinearOpMode {
    public static String[] motorName = {"", "", "", ""};

    public static double[] motorPower = new double[4];

    Motor[] motors = new Motor[4];

    @Override
    public void runOpMode() {
        for (int i = 0; i < 4; ++i)
            if (!motorName[i].isEmpty())
                motors[i] = new Motor(hardwareMap, motorName[i]);

        waitForStart();

        while (opModeIsActive()) {
            for (int i = 0; i < 4; ++i)
                if (!motorName[i].isEmpty())
                    motors[i].set(motorPower[i]);
        }
    }
}
