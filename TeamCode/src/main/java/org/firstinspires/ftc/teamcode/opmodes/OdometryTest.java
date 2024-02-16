package org.firstinspires.ftc.teamcode.opmodes;

import com.arcrobotics.ftclib.command.Robot;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Xordazh;

import java.io.FileWriter;
import java.io.IOException;

@TeleOp(name = "Test")
public class OdometryTest extends CommandOpMode {
    @Override
    public void initialize() {
        Robot m_robot = new Xordazh(this, Xordazh.InitMode.TEST);
    }
}
