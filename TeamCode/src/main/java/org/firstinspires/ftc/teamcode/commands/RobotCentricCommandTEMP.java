package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;

import org.firstinspires.ftc.teamcode.subsystems.drive.RobotCentricTEMP;

public class RobotCentricCommandTEMP extends CommandBase {
    private final RobotCentricTEMP robotCentricTEMP;

    private final GamepadEx gamepadEx;

    public RobotCentricCommandTEMP(RobotCentricTEMP robotCentricTEMP, GamepadEx gamepadEx) {
        this.robotCentricTEMP = robotCentricTEMP;
        this.gamepadEx = gamepadEx;

        addRequirements(robotCentricTEMP);
    }

    @Override
    public void execute() {
        double y = -gamepadEx.getLeftY();
        double x = gamepadEx.getLeftX();
        double rx = gamepadEx.getRightX();

        double frontLeftPower = (y - x - rx);
        double backLeftPower = (y + x - rx);
        double frontRightPower = (y + x + rx);
        double backRightPower = (y - x + rx);

        robotCentricTEMP.setDrive(frontLeftPower, backLeftPower, frontRightPower, backRightPower);

    }
}
