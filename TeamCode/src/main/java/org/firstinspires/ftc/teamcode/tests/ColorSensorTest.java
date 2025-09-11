package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;

@TeleOp(name="ColorSensor test")
public class ColorSensorTest extends LinearOpMode {
    private static ColorSensor colorSensor;

    public void runOpMode() {
        colorSensor = hardwareMap.get(ColorSensor.class, "colorsensor");

        while (opModeIsActive()) {

        }
    }

}
