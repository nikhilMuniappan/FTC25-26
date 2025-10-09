package org.firstinspires.ftc.teamcode.opmodes.testing_opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@Autonomous(name="AprilTagLocalization", group="Autonomous")
public class AprilTagLocalization extends LinearOpMode {

    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;

    @Override
    public void runOpMode() {

        aprilTag = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagID(true)
                .build();

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1")) // name must match RC config
                .addProcessor(aprilTag)
                .build();

        telemetry.addLine("AprilTag detector initialized.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // 3. Get AprilTag detections
            List<AprilTagDetection> detections = aprilTag.getDetections();

            if (detections.size() > 0) {
                AprilTagDetection tag = detections.get(0);

                double x = tag.ftcPose.x;     // x-pos relative to camera
                double y = tag.ftcPose.y;     // y-pos relative to camera
                double z = tag.ftcPose.z;     // z-pos relative to camera
                double yaw = tag.ftcPose.yaw; // heading rotation

                telemetry.addLine("AprilTag detected!");
                telemetry.addData("Tag ID", tag.id);
                telemetry.addData("X (in)", "%.2f", x);
                telemetry.addData("Y (in)", "%.2f", y);
                telemetry.addData("Z (in)", "%.2f", z);
                telemetry.addData("Heading (Yaw)", "%.2f deg", yaw);
            } else {
                telemetry.addLine("No AprilTags visible");
            }

            telemetry.update();
            sleep(20);
        }
    }
}
