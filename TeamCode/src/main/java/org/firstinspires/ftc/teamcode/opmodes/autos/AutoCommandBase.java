package org.firstinspires.ftc.teamcode.opmodes.autos;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.cds.CDS;
import org.firstinspires.ftc.teamcode.subsystems.drive.Constants;
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake;
import org.firstinspires.ftc.teamcode.subsystems.shooter.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.transit.Transit;

public abstract class AutoCommandBase extends LinearOpMode {
    protected Shooter shooter;
    protected Transit transit;
    protected Intake intake;
    protected CDS cds;
    protected Follower follower;

    public abstract Command runAutoCommand();

    public abstract Pose getStartPose();
    public abstract boolean highSpeed();

    private void initialize() {
        follower = Constants.createFollower(hardwareMap);

        follower.setStartingPose(getStartPose());
        shooter = new Shooter(hardwareMap, highSpeed());
        transit = new Transit(hardwareMap);
        intake = new Intake(hardwareMap, false);
        cds = new CDS(hardwareMap);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();

        Command toRun = runAutoCommand();//.andThen(autoFinish());

        CommandScheduler.getInstance().schedule(toRun);



        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            periodic();
        }

        onAutoStopped();
        CommandScheduler.getInstance().reset();
        TelemetryPacket packet = new TelemetryPacket();
        packet.put("shooterVelocity", shooter.getVelocity());
        FtcDashboard.getInstance().sendTelemetryPacket(packet);
    }

    /**
     *  The periodic function to update variables repeatedly。
     */
    public void periodic() {
        CommandScheduler.getInstance().run();
    }

    /**
     *  Executes when auto being stooped, either finished executing or stopped on DriverStation.
     */
    public void onAutoStopped() {}
}
