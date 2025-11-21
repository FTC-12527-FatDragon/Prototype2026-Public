package org.firstinspires.ftc.teamcode.opmodes.autos;


import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public abstract class AutoCommandBase extends LinearOpMode {
    public abstract Command runAutoCommand();

    @Override
    public void runOpMode() throws InterruptedException {
        Command toRun = runAutoCommand();//.andThen(autoFinish());

        CommandScheduler.getInstance().schedule(toRun);

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            periodic();
        }

        onAutoStopped();
        CommandScheduler.getInstance().reset();
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

    /**
     *  Executes when auto being initialized, but before runAutoCommand (starting).
     */
    public void initialize() {}
}
