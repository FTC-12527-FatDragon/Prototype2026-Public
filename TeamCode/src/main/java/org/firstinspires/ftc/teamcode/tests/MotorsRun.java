package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "MotorsRun")
@Config
public class MotorsRun extends LinearOpMode {
    public static String motor0Name = "";
    public static String motor1Name = "";
    public static String motor2Name = "";
    public static String motor3Name = "";

    public static double motor0Power;
    public static double motor1Power;
    public static double motor2Power;
    public static double motor3Power;

    public Motor motor0, motor1, motor2, motor3;

    @Override
    public void runOpMode() {
        if (!motor0Name.isEmpty()) motor0 = new Motor(hardwareMap, motor0Name);
        if (!motor1Name.isEmpty()) motor1 = new Motor(hardwareMap, motor1Name);
        if (!motor2Name.isEmpty()) motor2 = new Motor(hardwareMap, motor2Name);
        if (!motor3Name.isEmpty()) motor3 = new Motor(hardwareMap, motor3Name);

        waitForStart();

        while (opModeIsActive()) {
            if (!motor0Name.isEmpty()) motor0.set(motor0Power);
            if (!motor1Name.isEmpty()) motor1.set(motor1Power);
            if (!motor2Name.isEmpty()) motor2.set(motor2Power);
            if (!motor3Name.isEmpty()) motor3.set(motor3Power);
        }
    }
}
