package org.firstinspires.ftc.teamcode.tests;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "ShooterTestDot4")
public class ShooterTestDot4 extends LinearOpMode {
    Motor leftShooter;
    Motor rightShooter;

    @Override
    public void runOpMode() {

        leftShooter = new Motor(hardwareMap, "leftShooter");
        rightShooter = new Motor(hardwareMap, "rightShooter");

        waitForStart();

        while (opModeIsActive()) {
            rightShooter.set(-0.4);
            leftShooter.set(0.4);
        }

    }
}
