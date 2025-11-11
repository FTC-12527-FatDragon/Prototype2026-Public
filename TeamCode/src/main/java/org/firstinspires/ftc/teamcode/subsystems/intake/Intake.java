package org.firstinspires.ftc.teamcode.subsystems.intake;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.subsystems.transit.TransitConstants;

public class Intake extends SubsystemBase {
    public final DcMotor intakeMotor;

    public final Servo leftServo;
    public final Servo rightServo;

    public static boolean isRunning;

    public Intake(HardwareMap hardwareMap) {
        intakeMotor = hardwareMap.get(DcMotor.class, IntakeConstants.intakeMotorName);
        leftServo = hardwareMap.get(Servo.class, IntakeConstants.leftServoName);
        rightServo = hardwareMap.get(Servo.class, IntakeConstants.rightServoName);


        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        leftServo.setDirection(Servo.Direction.REVERSE);

        isRunning = false;
    }

    public void toggle() {
        isRunning = !isRunning;
    }

    public boolean isRunning() {
        return isRunning;
    }

    @Override
    public void periodic() {
        if (isRunning) {
            intakeMotor.setPower(IntakeConstants.intakePower);
            leftServo.setPosition(0);
            rightServo.setPosition(1);
        }
        else {
            intakeMotor.setPower(0);
            leftServo.setPosition(0.5);
            rightServo.setPosition(0.5);
        }


    }
}
