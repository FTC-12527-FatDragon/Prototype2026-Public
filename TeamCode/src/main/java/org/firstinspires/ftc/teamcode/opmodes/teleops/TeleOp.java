package org.firstinspires.ftc.teamcode.opmodes.teleops;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.bylazar.configurables.annotations.Configurable;

import org.firstinspires.ftc.teamcode.commands.HeadResetCommand;
import org.firstinspires.ftc.teamcode.commands.ShootCommand;
import org.firstinspires.ftc.teamcode.commands.TeleopDriveCommand;
import org.firstinspires.ftc.teamcode.subsystems.drive.Drive;
import org.firstinspires.ftc.teamcode.subsystems.shooter.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.shooter.ShooterSpeedCalc;
import org.firstinspires.ftc.teamcode.utils.FunctionalButton;

@Configurable
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOpCY")
public class TeleOp extends CommandOpMode {
    private Drive drive;

    private GamepadEx gamepadEx1;

    private Shooter shooter;

    @Override
    public void initialize() {
        drive = new Drive(hardwareMap);
        gamepadEx1 = new GamepadEx(gamepad1);
        shooter = new Shooter(hardwareMap);

        drive.setDefaultCommand(new TeleopDriveCommand(drive, gamepadEx1));

        new FunctionalButton(
                () -> gamepadEx1.getButton(GamepadKeys.Button.LEFT_STICK_BUTTON)
        ).whenPressed(
                new HeadResetCommand(drive)
        );

        new FunctionalButton(
                () -> gamepadEx1.getButton(GamepadKeys.Button.A)
        ).whenHeld(
                new ShootCommand(shooter, ShooterSpeedCalc.calcSpeed(drive.getPose()))
        );
    }

    @Override
    public void run() {
        CommandScheduler.getInstance().run();
    }
}
