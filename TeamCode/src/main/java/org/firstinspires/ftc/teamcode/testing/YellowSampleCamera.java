package org.firstinspires.ftc.teamcode.testing;
import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.teamcode.library.Subsystem;
import org.firstinspires.ftc.teamcode.vision.YellowSampleDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;
import org.openftc.easyopencv.OpenCvWebcam.StreamFormat;

public class YellowSampleCamera extends Subsystem {

    private OpenCvWebcam camera;
    private YellowSampleDetection pipeline;   // single pipeline instance
    private final Telemetry telemetry;
    private boolean on = false;

    public YellowSampleCamera(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        int viewId = hardwareMap.appContext.getResources()
                .getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance()
                .createWebcam(hardwareMap.get(WebcamName.class, RobotConstants.camera), viewId);

        // create the pipeline once here
        pipeline = new YellowSampleDetection();
    }

    @Override
    public void init() {
        camera.setPipeline(pipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override public void onOpened() { startCamera(); }
            @Override public void onError(int errorCode) {
                telemetry.addData("Camera2_0", "open error: " + errorCode);
                telemetry.update();
            }
        });
    }

    public void startCamera() {
        on = true;
        camera.startStreaming(960, 544, OpenCvCameraRotation.UPRIGHT, StreamFormat.MJPEG);
        FtcDashboard.getInstance().startCameraStream(camera, 5);
    }

    public void stopCamera() {
        on = false;
        camera.stopStreaming();
    }

    public boolean isCameraOn() { return on; }

    public double getDistanceToYellowSample() {
        return pipeline.getDistanceInches();
    }

    public boolean isSampleDetected() {
        return pipeline.isSampleDetected();
    }

    public boolean IsYellowSampleDetected() {
        return isSampleDetected();
    }

    public int getPipelineIdentity() {
        return System.identityHashCode(pipeline);
    }
    public long getPipelineFrameCount() {
        return pipeline.getFrameCount();
    }

    @Override public void telemetry() {
        telemetry.addData("Yellow detected", isSampleDetected());
        telemetry.addData("Yellow distance (in)", getDistanceToYellowSample());
        telemetry.addData("Pipeline id", getPipelineIdentity());
        telemetry.addData("Frames processed", getPipelineFrameCount());
        telemetry.update();
    }

    @Override public void update() { }

    public class WaitForYellowSampleAction implements Action {
        ElapsedTime timer = new ElapsedTime();
        boolean first = true;
        @Override public boolean run(@NonNull TelemetryPacket p) {
            if (first) { timer.reset(); first = false; }
            if (timer.seconds() > 2.0) return false;
            return !isSampleDetected();
        }
    }

    public Action waitForYellowSample() { return new WaitForYellowSampleAction(); }
}