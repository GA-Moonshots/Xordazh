package org.firstinspires.ftc.teamcode.config;

import com.acmerobotics.dashboard.config.Config;

@Config
public class OdometryConfig {
    public static volatile double MM_PER_TICK = 48 * Math.PI / 2000;
    public static volatile double CM_PER_TICK = MM_PER_TICK / 10;
    public static volatile double IN_PER_TICK = CM_PER_TICK / 2.54;
    public static volatile double PARALLEL_DISTANCE = 197 / MM_PER_TICK;
    public static volatile double PERPENDICULAR_DISTANCE = 170 / MM_PER_TICK;
}
