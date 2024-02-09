package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.Robot;

import org.firstinspires.ftc.teamcode.config.InputConfig;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDrive;

public class DriveCommand extends CommandBase {
    private MecanumDrive drive;

    public DriveCommand(MecanumDrive drive) {
        this.drive = drive;

        addRequirements(drive);
    }

    @Override
    public void initialize() {
        // Runs before anything else (besides constructor ofc) from this command is run.
    }

    private double applyDeadZone(double input) {
        return Math.abs(input) <= InputConfig.INPUT_THRESHOLD ? 0.0d : input;
    }
}
