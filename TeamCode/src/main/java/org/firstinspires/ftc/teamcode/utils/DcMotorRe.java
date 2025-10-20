package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.ArrayDeque;
import java.util.Deque;

public class DcMotorRe{
    private final DcMotor motor;
    private double lastPos = 0;
    private final double WINDOW = 10;
    private final Deque<Double> posList = new ArrayDeque<>();

    public DcMotorRe(final HardwareMap hardwareMap, final String motorName) {
        motor = hardwareMap.get(DcMotor.class, motorName);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public double getPosition() { return motor.getCurrentPosition(); }

    /**
     * In Ticks Per Second
     */
    public double getInstantVelocity() {
        return (motor.getCurrentPosition() - lastPos) / 0.02;
    }

    public double getAverageVelocity() {
        if (posList.peekFirst() != null && posList.peekLast() != null)
            return (posList.peekLast() - posList.peekFirst()) / (WINDOW * 0.02);
        return 0;
    }

    public void setPower(double power) {
        motor.setPower(power);
    }

    public void updateLastPos(){
        lastPos = motor.getCurrentPosition();
        if (posList.element() >= WINDOW) {
            posList.removeFirst();
        }
        posList.addLast(lastPos);
    }
}
