package org.firstinspires.ftc.teamcode.utils;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.ArrayDeque;
import java.util.Deque;

public class DcMotorRe {
    private final DcMotorEx motor;
    private double lastPos = 0;
    private final double WINDOW = 10;
    private final Deque<Double> posList = new ArrayDeque<>();

    public DcMotorRe(final HardwareMap hardwareMap, final String motorName) {
        motor = hardwareMap.get(DcMotorEx.class, motorName);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public double getPosition() {
        return motor.getCurrentPosition();
    }

    /**
     * In Ticks Per Second
     */
    public double getInstantVelocity() {
        return (motor.getCurrentPosition() - lastPos) / 0.02;
    }

    /**
     * In Ticks Per Second
     */
    public double getAverageVelocity() {
        if (posList.peekFirst() != null && posList.peekLast() != null)
            return (posList.peekLast() - posList.peekFirst()) / (WINDOW * 0.02);
        return 0;
    }

    public void setPower(double power) {
        motor.setPower(power);
    }

    public double getLibVelocity() {
        return motor.getVelocity();
    }

    public void updateLastPos() {
        lastPos = motor.getCurrentPosition();
        if (posList.size() >= WINDOW) {
            posList.removeFirst();
        }
        posList.addLast(lastPos);
    }
}
