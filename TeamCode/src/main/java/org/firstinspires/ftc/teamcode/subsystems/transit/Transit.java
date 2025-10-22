package org.firstinspires.ftc.teamcode.subsystems.transit;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Transit extends SubsystemBase {
    public final DcMotorEx transit;

    public Transit(HardwareMap hardwareMap) {
        transit = hardwareMap.get(DcMotorEx.class, TransitConstants.transitName);
    }

    public void setPower(double power) {
        transit.setPower(power);
    }
}
