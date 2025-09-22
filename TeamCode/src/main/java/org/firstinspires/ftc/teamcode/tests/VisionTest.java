package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.bylazar.configurables.annotations.Configurable;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.vision.AutoApriltag;

import java.util.List;

@Configurable
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "VisionTest")
public class VisionTest extends CommandOpMode {
    private AutoApriltag autoApriltag;

    private Telemetry telemetryM;
    @Override
    public void initialize() {
        autoApriltag = new AutoApriltag(hardwareMap);
    }

    @Override
    public void run() {
        List ids = autoApriltag.getNumbers();
        telemetryM = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        CommandScheduler.getInstance().run();
        telemetry.addData("ApriltagID",ids);
        telemetry.update();
    }
}
