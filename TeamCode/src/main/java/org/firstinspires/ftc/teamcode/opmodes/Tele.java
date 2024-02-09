package org.firstinspires.ftc.teamcode.opmodes;


import com.arcrobotics.ftclib.command.Robot;

import org.firstinspires.ftc.teamcode.Xordazh;

public class Tele extends CommandOpMode {

    @Override
    public void initialize() {
        Robot m_robot = new Xordazh(this);
    }
}
