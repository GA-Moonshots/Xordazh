package org.firstinspires.ftc.teamcode.opmodes;

import com.arcrobotics.ftclib.command.Robot;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Xordazh;

@TeleOp(name = "Main")
public class Main extends CommandOpMode {

    @Override
    public void initialize() {
        Robot m_robot = new Xordazh(this, Xordazh.InitMode.TELEOP);
    }
}
