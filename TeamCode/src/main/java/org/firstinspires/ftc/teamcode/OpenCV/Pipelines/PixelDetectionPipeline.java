package org.firstinspires.ftc.teamcode.OpenCV.Pipelines;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class PixelDetectionPipeline extends OpenCvPipeline {


    private Scalar lowerGreen = new Scalar(35, 100, 50);
    private Scalar upperGreen = new Scalar(85, 255, 255);

    private Scalar lowerYellow = new Scalar(20, 100, 100);
    private Scalar upperYellow = new Scalar(30, 255, 255);

    private Scalar lowerPurple = new Scalar(145, 100, 100);
    private Scalar upperPurple = new Scalar(171, 255, 255);

    private Scalar lowerWhite = new Scalar(75, 0, 99);
    private Scalar upperWhite = new Scalar(179, 62, 255);

    @Override
    public Mat processFrame(Mat input) {
        Mat inputToHSV = new Mat();

        Mat justGreenDetected = new Mat();
        Mat justYellowDetected = new Mat();
        Mat justPurpleDetected = new Mat();
        Mat justWhiteDetected = new Mat();

        Mat output = new Mat();

        Imgproc.cvtColor(input, inputToHSV, Imgproc.COLOR_RGB2HSV);

        Core.inRange(inputToHSV, lowerGreen, upperGreen, justGreenDetected);
        Core.inRange(inputToHSV, lowerYellow, upperYellow, justYellowDetected);
        Core.inRange(inputToHSV, lowerPurple, upperPurple, justPurpleDetected);
        Core.inRange(inputToHSV, lowerWhite, upperWhite, justWhiteDetected);


        Core.bitwise_and(input, input, output, justGreenDetected);
        Core.bitwise_and(input, input, output, justPurpleDetected);
        Core.bitwise_and(input, input, output, justYellowDetected);
        Core.bitwise_and(input, input, output, justWhiteDetected);

        List<MatOfPoint> contoursYellow = new ArrayList<>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(justYellowDetected, contoursYellow, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);


        for (MatOfPoint contour : contoursYellow) {
            Imgproc.drawContours(output, contoursYellow, -1, new Scalar(0, 255, 255), 2);
        }

        Core.bitwise_and(output, output, input);

        inputToHSV.release();
        justGreenDetected.release();
        justPurpleDetected.release();
        justYellowDetected.release();
        justWhiteDetected.release();

        hierarchy.release();
        output.release();

        return input;
    }
}
