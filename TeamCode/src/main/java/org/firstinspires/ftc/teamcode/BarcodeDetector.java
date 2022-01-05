package org.firstinspires.ftc.teamcode;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class BarcodeDetector extends OpenCvPipeline {
    private Mat mat = new Mat();
    public enum BarcodePosition {
        ONE,
        TWO,
        THREE
    }

    public volatile BarcodePosition position = BarcodePosition.ONE;

    public BarcodeDetector() {

    }

    @Override
    public Mat processFrame(Mat input) {
        input.copyTo(mat);

        if (mat.empty()) {
            return input;
        }

        Imgproc.cvtColor(mat, mat, Imgproc.COLOR_RGB2YCrCb);

        Mat matLeft = mat.submat(120, 150, 10, 50);
        Mat matCenter = mat.submat(120, 150, 80, 120);
        Mat matRight = mat.submat(120, 150, 150, 190);

        Imgproc.rectangle(mat, new Rect(10, 120, 40, 30), new Scalar(0, 255, 0));
        Imgproc.rectangle(mat, new Rect(80, 120, 40, 30), new Scalar(0, 255, 0));
        Imgproc.rectangle(mat, new Rect(150, 120, 40, 30), new Scalar(0, 255, 0));

        double leftTotal = Core.sumElems(matLeft).val[2];
        double centerTotal = Core.sumElems(matCenter).val[2];
        double rightTotal = Core.sumElems(matRight).val[2];

        if (leftTotal < rightTotal && leftTotal < centerTotal) {
            position = BarcodePosition.ONE;
        } else if (rightTotal < leftTotal && rightTotal < centerTotal) {
            position = BarcodePosition.TWO;
        } else {
            position = BarcodePosition.THREE;
        }

        return mat;
    }
}
