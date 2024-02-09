package org.firstinspires.ftc.teamcode.config;

import com.acmerobotics.dashboard.config.Config;

@Config
public class OdometryConfig {
    public static volatile double MM_PER_TICK = 48 * Math.PI / 2000;
    public static volatile double PARALLEL_DISTANCE = 200;
    public static volatile double PERPENDICULAR_DISTANCE = 170;
}
