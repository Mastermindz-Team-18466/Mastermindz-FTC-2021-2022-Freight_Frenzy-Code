package org.firstinspires.ftc.teamcode.vision;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class BarcodeDetector extends OpenCvPipeline {
    private final Mat mat = new Mat();

    boolean safe = true;

    public enum BarcodePosition {
        DEFAULT,
        ONE,
        TWO,
        THREE
    }

    public BarcodePosition position = BarcodePosition.DEFAULT;

    @Override
    public Mat processFrame(Mat input) {
        input.copyTo(mat);

        if (mat.empty()) {
            return input;
        }

        Imgproc.cvtColor(mat, mat, Imgproc.COLOR_RGB2YCrCb);

        Mat matLeft = mat.submat(0, 240, 0, 106);
        Mat matCenter = mat.submat(0, 240, 106, 213);
        Mat matRight = mat.submat(0, 240, 213, 320);

        Imgproc.rectangle(mat, new Rect(0, 0, 106, 240), new Scalar(0, 255, 0));
        Imgproc.rectangle(mat, new Rect(106, 0, 107, 240), new Scalar(0, 255, 0));
        Imgproc.rectangle(mat, new Rect(213, 0, 106, 240), new Scalar(0, 255, 0));

        double leftTotal = Core.sumElems(matLeft).val[2];
        double centerTotal = Core.sumElems(matCenter).val[2];
        double rightTotal = Core.sumElems(matRight).val[2];

        if (leftTotal < centerTotal && leftTotal < rightTotal && safe == true) {
            position = BarcodePosition.ONE;
            safe = false;
        }

        else if (centerTotal < rightTotal && centerTotal < leftTotal && safe == true) {
            position = BarcodePosition.TWO;
            safe = false;
        }

        else if (rightTotal < centerTotal && rightTotal < leftTotal && safe == true) {
            position = BarcodePosition.THREE;
            safe = false;
        }

        return mat;
    }
}
