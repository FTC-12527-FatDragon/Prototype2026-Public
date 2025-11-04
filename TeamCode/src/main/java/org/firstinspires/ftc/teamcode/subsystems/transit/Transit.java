package org.firstinspires.ftc.teamcode.subsystems.transit;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Transit extends SubsystemBase {
    public final DcMotor transit;

    public final Servo leftTransitServo;
    public final Servo rightTransitServo;
    public final Servo limitServo;

    public double transitPower = TransitState.STOP.power;
    public double transitServoPower = 0.5;
    public double limitServoPos = LimitServoState.CLOSE.servoPos;
    public double stopTime = 0;



    public enum LimitServoState {
        CLOSE(TransitConstants.limitServoClosePos),
        OPEN(TransitConstants.limitServoOpenPos);

        double servoPos;

        LimitServoState(double limitServoPos) {
            servoPos = limitServoPos;
        }
    }

    public enum TransitState {
        STOP(TransitConstants.transitStopPower),
        INTAKE(TransitConstants.transitIntakePower),
        SHOOT(TransitConstants.transitShootPower);

        double power;

        TransitState(double transitPower) {
            power = transitPower;
        }
    }

    public Transit(HardwareMap hardwareMap) {
        transit = hardwareMap.get(DcMotor.class, TransitConstants.transitName);
        leftTransitServo = hardwareMap.get(Servo.class, TransitConstants.leftTransitServoName);
        rightTransitServo = hardwareMap.get(Servo.class, TransitConstants.rightTransitServoName);
        limitServo = hardwareMap.get(Servo.class, TransitConstants.limitServoName);

        leftTransitServo.setDirection(Servo.Direction.REVERSE);
    }

    public void setLimitServoState(LimitServoState limitServoState) {
        limitServoPos = limitServoState.servoPos;
    }

    public void transitServoOn() {
        transitServoPower = 1;
    }

    public void transitServoOff() {
        transitServoPower = 0.5;
    }

    public void setPower(double power) {
        transitPower = power;
    }

    public void setTransitState(TransitState transitState) {
        transitPower = transitState.power;
    }

    public void stopTransit(double time) {
        setTransitState(TransitState.STOP);
        stopTime = time;
    }

    @Override
    public void periodic() {
        if (stopTime <= 0) transit.setPower(transitPower);
        else transit.setPower(0);
        leftTransitServo.setPosition(transitServoPower);
        rightTransitServo.setPosition(transitServoPower);
        limitServo.setPosition(limitServoPos);
        if (stopTime > 0) stopTime -= 20;
    }
}
