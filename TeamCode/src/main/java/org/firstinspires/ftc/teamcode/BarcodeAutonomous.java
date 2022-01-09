package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.vision.BarcodeDetector;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

@Autonomous(name = "Barcode Autonomous")
public class BarcodeAutonomous extends LinearOpMode {
    Hardware hardware = new Hardware();
    private BarcodeDetector detector;

    @Override
    public void runOpMode() throws InterruptedException {
        hardware.init(hardwareMap);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        hardware.webcam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        hardware.webcam.openCameraDevice();
        hardware.webcam.setPipeline(detector);
        hardware.webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);

        while (opModeIsActive()) {
            switch (detector.position) {
                case ONE:
                    break;
                case TWO:
                    break;
                case THREE:
                    break;
            }

            telemetry.addData("Position", detector.position);
            telemetry.update();

            sleep(50);
        }
    }
}
