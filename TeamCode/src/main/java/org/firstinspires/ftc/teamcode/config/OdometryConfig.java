package org.firstinspires.ftc.teamcode.config;

import com.acmerobotics.dashboard.config.Config;

@Config
public class OdometryConfig {
    public static volatile double MM_PER_TICK = 48 * Math.PI / 2000;
    public static volatile double CM_PER_TICK = MM_PER_TICK / 10;
    public static volatile double IN_PER_TICK = CM_PER_TICK / 2.54;
    public static volatile double PARALLEL_DISTANCE_MM = 200;
    public static volatile double PARALLEL_DISTANCE_CM = PARALLEL_DISTANCE_MM / 10;
    public static volatile double PARALLEL_DISTANCE_IN = PARALLEL_DISTANCE_CM / 2.54;
    public static volatile double PERPENDICULAR_DISTANCE_MM = 170;
    public static volatile double PERPENDICULAR_DISTANCE_CM = PERPENDICULAR_DISTANCE_MM / 10;
    public static volatile double PERPENDICULAR_DISTANCE_IN = PERPENDICULAR_DISTANCE_CM / 2.54;
}
