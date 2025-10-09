package org.firstinspires.ftc.teamcode.vision;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class YellowSampleDetection extends OpenCvPipeline {

    private static final double KNOWN_WIDTH_INCHES = 2.0;

    private final int IMAGE_WIDTH = 960;
    private final int IMAGE_HEIGHT = 544;
    private final double DIAGONAL_FOV_DEGREES = 55.0;
    private final double focalLengthPx;

    private double distanceInches = -1;
    private boolean sampleDetected = false;
    private volatile long frameCount = 0;
    private final Scalar lowerHSV = new Scalar(20, 100, 100);
    private final Scalar upperHSV = new Scalar(30, 255, 255);

    public YellowSampleDetection() {
        double diagPx = Math.hypot(IMAGE_WIDTH, IMAGE_HEIGHT);
        this.focalLengthPx = (diagPx / 2.0) /
                Math.tan(Math.toRadians(DIAGONAL_FOV_DEGREES / 2.0));
    }

    public double getDistanceInches() {
        return distanceInches;
    }

    public boolean isSampleDetected() {
        return sampleDetected;
    }
    public long getFrameCount() {
        return frameCount;
    }


    @Override
    public Mat processFrame(Mat input) {
        Mat hsv = new Mat();
        Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);

        Mat mask = new Mat();
        Core.inRange(hsv, lowerHSV, upperHSV, mask);

        Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(5, 5));
        Imgproc.erode(mask, mask, kernel);
        Imgproc.dilate(mask, mask, kernel);

        List<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(mask, contours, new Mat(),
                Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        double maxArea = 0;
        Rect largestRect = null;

        for (MatOfPoint contour : contours) {
            Rect rect = Imgproc.boundingRect(contour);
            double area = rect.width * rect.height;
            if (area > maxArea) {
                maxArea = area;
                largestRect = rect;
            }
        }

        if (largestRect != null && largestRect.width > 0) {
            double perceivedWidthPx = largestRect.width;
            distanceInches = (KNOWN_WIDTH_INCHES * focalLengthPx) / perceivedWidthPx;
            sampleDetected = true;
            Imgproc.rectangle(input, largestRect, new Scalar(0, 255, 0), 2);
        } else {
            distanceInches = -1.0;
            sampleDetected = false;
        }

        hsv.release();
        mask.release();
        kernel.release();

        return input;
    }
}