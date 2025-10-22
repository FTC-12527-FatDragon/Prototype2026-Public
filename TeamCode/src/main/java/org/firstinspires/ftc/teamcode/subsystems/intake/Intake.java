package org.firstinspires.ftc.teamcode.subsystems.intake;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake extends SubsystemBase {
    public final DcMotorEx intakeMotor;

    public static boolean isRunning;

    public Intake(HardwareMap hardwareMap) {
        intakeMotor = hardwareMap.get(DcMotorEx.class, IntakeConstants.intakeMotorName);

        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);

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
        }
        else {
            intakeMotor.setPower(0);
        }
    }
}
