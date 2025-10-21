package org.firstinspires.ftc.teamcode.subsystems.vision;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ColorSensor extends SubsystemBase {
    private final ColorSensor colorSensor;
    private final DistanceSensor distanceSensor;

    public ColorSensor(HardwareMap hardwareMap) {
        colorSensor = hardwareMap.get(ColorSensor.class, VisionConstants.colorSensorName);
        distanceSensor = hardwareMap.get(DistanceSensor.class, VisionConstants.distanceSensorName);
    }

    @Override
    public void periodic() {

    }
}
