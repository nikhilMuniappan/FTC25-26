package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.LazyImu;

@Config
public class DECODERobotConstants {
    public static String fr = "frontRight";
    public static String br = "backRight";
    public static String fl = "frontLeft";
    public static String bl = "backLeft";
    public static LazyImu imu;
    public static boolean imu_init = false;

    public static Pose2d pose = new Pose2d(0,0,0);
    public static double wrist_folded = 0, wrist_extended=30;
    public static String rollers = "rollers";
    public static String flywheels = "flywheels";

}
