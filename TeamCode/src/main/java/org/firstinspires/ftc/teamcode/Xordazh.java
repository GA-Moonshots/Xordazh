package org.firstinspires.ftc.teamcode;

import android.os.Environment;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.Robot;
import com.arcrobotics.ftclib.command.button.Button;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.commands.DriveCommand;
import org.firstinspires.ftc.teamcode.commands.OdometryTest;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDrive;

import java.io.FileWriter;
import java.io.IOException;

public class Xordazh extends Robot {
    public LinearOpMode opMode;
    public FileWriter writer;

    private GamepadEx player1;
    public GamepadEx getPlayer1() {
        return player1;
    }
    private GamepadEx player2;
    public GamepadEx getPlayer2() {
        return player2;
    }

    private MecanumDrive drive;
    public MecanumDrive getDrive() {
        return drive;
    }

    public enum InitMode {
        TELEOP,
        AUTONOMOUS,
        TEST,
    }

    public Xordazh(LinearOpMode opMode, InitMode initMode) {
        this.opMode = opMode;
        player1 = new GamepadEx(opMode.gamepad1);
        player2 = new GamepadEx(opMode.gamepad2);

        switch(initMode) {
            case TEST:
                initTest();
            case AUTONOMOUS:
                initAuto(false, false, false);
                break;
            case TELEOP:
            default:
                initTele();
                break;
        }
    }

    public Xordazh(LinearOpMode opMode, boolean isRed, boolean isLeft, boolean goForBoard) {
        this.opMode = opMode;
        initAuto(isRed, isLeft, goForBoard);
    }

    public void initTest() {
        drive = new MecanumDrive(this, new Pose2d(0,0,new Rotation2d(1, 0)));
        this.opMode.telemetry.addData("Status", "Initialize Finished");
        this.opMode.telemetry.update();
        register(drive);
        drive.setDefaultCommand(new OdometryTest(this));

    }

    /**
     * Set teleOp's default commands and player control bindings
     */
    private void initTele() {
        // throw-away pose because we're not localizing anymore
        drive = new MecanumDrive(this, new Pose2d(0,0,new Rotation2d(1, 0)));
        this.opMode.telemetry.addData("Status", "Initialize Finished");
        this.opMode.telemetry.update();
        register(drive);
        drive.setDefaultCommand(new DriveCommand(this));

        /*
                .__                                      ____
        ______  |  |  _____   ___.__.  ____ _______     /_   |
        \____ \ |  |  \__  \ <   |  |_/ __ \\_  __ \     |   |
        |  |_> >|  |__ / __ \_\___  |\  ___/ |  | \/     |   |
        |   __/ |____/(____  // ____| \___  >|__|        |___|
        |__|               \/ \/          \/
        */
        Button aButtonP1 = new GamepadButton(player1, GamepadKeys.Button.A);
        aButtonP1.whenPressed(new InstantCommand(() -> {
            if(!this.opMode.gamepad1.start)
                drive.toggleFieldCentric();
        }));

        Button bButtonP1 = new GamepadButton(player1, GamepadKeys.Button.B);
        bButtonP1.whenPressed(new InstantCommand(() -> {
            if(!this.opMode.gamepad1.start)
                drive.resetFieldCentricTarget();
        }));

        /*
                _                                    __
               (_ )                                /'__`\
         _ _    | |    _ _  _   _    __   _ __    (_)  ) )
        ( '_`\  | |  /'_` )( ) ( ) /'__`\( '__)      /' /
        | (_) ) | | ( (_| || (_) |(  ___/| |       /' /( )
        | ,__/'(___)`\__,_)`\__, |`\____)(_)      (_____/'
        | |                ( )_| |
        (_)                `\___/'
         */

    }

    private void initAuto(boolean isRed, boolean isLeft, boolean goForBoard) {

    }
}
