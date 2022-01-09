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

    public enum BarcodePosition {
        ONE,
        TWO,
        THREE
    }

    public volatile BarcodePosition position = BarcodePosition.THREE;
    Telemetry telemetry;

    public BarcodeDetector(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    @Override
    public Mat processFrame(Mat input) {
        input.copyTo(mat);

        if (mat.empty()) {
            return input;
        }

        Imgproc.cvtColor(mat, mat, Imgproc.COLOR_RGB2YCrCb);

        Mat matLeft = mat.submat(35, 65, 30, 57);
        Mat matCenter = mat.submat(35, 65, 125, 152);
        Mat matRight = mat.submat(35, 65, 220, 247);

        Imgproc.rectangle(mat, new Rect(30, 35, 27, 30), new Scalar(0, 255, 0));
        Imgproc.rectangle(mat, new Rect(125, 35, 27, 30), new Scalar(0, 255, 0));
        Imgproc.rectangle(mat, new Rect(220, 35, 27, 30), new Scalar(0, 255, 0));

        double leftTotal = Core.sumElems(matLeft).val[2];
        double centerTotal = Core.sumElems(matCenter).val[2];
        double rightTotal = Core.sumElems(matRight).val[2];

        if (leftTotal < centerTotal) {
            if (leftTotal < rightTotal) {
                position = BarcodePosition.ONE;
            } else {
                position = BarcodePosition.THREE;
            }
        } else {
            if (centerTotal < rightTotal) {
                position = BarcodePosition.TWO;
            } else {
                position = BarcodePosition.THREE;
            }
        }

        telemetry.addData("[Position]", position);
        telemetry.addData("[Left]", leftTotal);
        telemetry.addData("[Center]", centerTotal);
        telemetry.addData("[Right]", rightTotal);
        telemetry.update();

        return mat;
    }
}
