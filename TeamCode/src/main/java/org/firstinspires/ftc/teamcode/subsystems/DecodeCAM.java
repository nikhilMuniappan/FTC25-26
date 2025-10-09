package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.teamcode.library.Subsystem;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import java.util.List;

public class DecodeCAM extends Subsystem{
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTagProcessor;
    OpenCvCamera camera;
    Telemetry telemetry;

    public void init(android.content.Context appContext, com.qualcomm.robotcore.hardware.HardwareMap hardwareMap) {
        if (aprilTagProcessor == null) {
            aprilTagProcessor = new AprilTagProcessor.Builder()
                    .setCameraPose(
                            new Position(DistanceUnit.INCH, 0, 0, 0, 0),
                            new YawPitchRollAngles(AngleUnit.DEGREES, 0, -90, 0, 0)
                    )
                    .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                    .setTagLibrary(org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase.getCurrentGameTagLibrary())
                    .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                    .build();

            visionPortal = new VisionPortal.Builder()
                    .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                    .addProcessor(aprilTagProcessor)
                    .build();
        }
    }

    public String getMotif() {
        if (aprilTagProcessor == null) return "Not Initialized";

        List<AprilTagDetection> detections = aprilTagProcessor.getDetections();
        if (detections.isEmpty()) return "No Tag";

        for (AprilTagDetection tag : detections) {
            switch (tag.id) {
                case 21: return "GPP";
                case 22: return "PGP";
                case 23: return "PPG";
            }
        }
        return "Unknown Motif";
    }

    public void stop() {
        if (visionPortal != null) {
            visionPortal.close();
        }
    }

    @Override
    public void update() {

    }

    @Override
    public void telemetry() {

    }

    @Override
    public void init() {
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(960, 544, OpenCvCameraRotation.UPRIGHT);
                FtcDashboard.getInstance().startCameraStream(camera, 5);
            }

            @Override
            public void onError(int errorCode) {
            }
        });
    }
}
