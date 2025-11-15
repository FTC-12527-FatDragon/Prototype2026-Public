package org.firstinspires.ftc.teamcode.subsystems.transit;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Transit extends SubsystemBase {
    public final DcMotor transit;

    public final Servo limitServo;

    public final Servo chooseServo;

    public TransitState transitState = TransitState.STOP;
    public LimitServoState limitState = LimitServoState.CLOSE;
    public ChooseServoState chooseState = ChooseServoState.CLOSE;
    public static double transitPower = 0;
    public double stopTime = 0;

    public Transit(HardwareMap hardwareMap) {
        transit = hardwareMap.get(DcMotor.class, TransitConstants.transitName);

        limitServo = hardwareMap.get(Servo.class, TransitConstants.limitServoName);

        chooseServo = hardwareMap.get(Servo.class, TransitConstants.chooseServoName);
    }

    public enum LimitServoState {
        CLOSE(TransitConstants.limitServoClosePos),
        OPEN(TransitConstants.limitServoOpenPos);

        final double servoPos;

        LimitServoState(double limitServoPos) {
            servoPos = limitServoPos;
        }
    }

    public enum ChooseServoState {
        CLOSE(TransitConstants.chooseServoClosePos),
        OPEN(TransitConstants.chooseServoOpenPos);

        final double chooseServoPos;

        ChooseServoState(double chooseServoPos) {
            this.chooseServoPos = chooseServoPos;
        }
    }

    public enum TransitState {
        STOP(TransitConstants.transitStopPower),
        INTAKE(TransitConstants.transitIntakePower),
        SHOOT(TransitConstants.transitShootPower),
        OPENLOOP(transitPower);

        final double power;

        TransitState(double transitPower) {
            power = transitPower;
        }
    }


    public void setLimitServoState(LimitServoState limitServoState) {
        limitState = limitServoState;
    }

    public void setPower(double power) {
        transitState = TransitState.OPENLOOP;
        transitPower = power;
    }

    public void setTransitState(TransitState transitState) {
        transitPower = transitState.power;
    }

    public void setChooseServoState(ChooseServoState chooseServoState) {
        chooseState = chooseServoState;
        limitState = chooseServoState == ChooseServoState.CLOSE?
                LimitServoState.CLOSE:LimitServoState.OPEN;
    }

    public void stopTransit(double time) {
        setTransitState(TransitState.STOP);
        stopTime = time;
    }

    @Override
    public void periodic() {
        if (stopTime <= 0) transit.setPower(transitPower);
        else transit.setPower(0);
        limitServo.setPosition(limitState.servoPos);
        chooseServo.setPosition(chooseState.chooseServoPos);

        if (stopTime > 0) stopTime -= 20;
    }
}
