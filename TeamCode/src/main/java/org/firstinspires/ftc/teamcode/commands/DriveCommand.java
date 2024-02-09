package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.Robot;

import org.firstinspires.ftc.teamcode.Xordazh;
import org.firstinspires.ftc.teamcode.config.InputConfig;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDrive;

public class DriveCommand extends CommandBase {
    private final MecanumDrive drive;
    private final Xordazh robot;

    public DriveCommand(Xordazh robot) {
        this.drive = robot.getDrive();
        this.robot = robot;

        addRequirements(drive);
    }

    @Override
    public void initialize() {
        // Runs before anything else (besides constructor ofc) from this command is run.
    }

    private double applyDeadZone(double input) {
        return Math.abs(input) <= InputConfig.INPUT_THRESHOLD ? 0.0d : input;
    }

    @Override
    public void execute() {

        // you can go digging for the opMode controller
        double speedMod = robot.getPlayer1().gamepad.right_bumper ? 0.2 : 1; // slow mode

        double forward = -applyDeadZone(robot.getPlayer1().getLeftY());
        double strafe = applyDeadZone(robot.getPlayer1().getLeftX());
        double turn = applyDeadZone(robot.getPlayer1().getRightX());

        // Drive the robot with adjusted inputs:
        drive.drive(forward * speedMod, strafe * speedMod, turn * speedMod);

    }
}
