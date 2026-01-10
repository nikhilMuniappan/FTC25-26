package org.firstinspires.ftc.teamcode.roadrunner;
import com.acmerobotics.roadrunner.Pose2d;

public class PoseStorage {
    static Pose2d blueAutoEndPose;
    static Pose2d redAutoEndPose;
    public static void storeBluePose(Pose2d pose){
        blueAutoEndPose = pose;
    }
    public static Pose2d getLastBluePose() { return blueAutoEndPose; }
    public static void storeRedPose(Pose2d pose){
        redAutoEndPose = pose;
    }
    public static Pose2d getLastRedPose(){
        return redAutoEndPose;
    }
    public static void resetPose() {
        blueAutoEndPose = null;
        redAutoEndPose = null;
    }

}
