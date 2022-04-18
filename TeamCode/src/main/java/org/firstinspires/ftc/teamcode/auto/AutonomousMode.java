package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.vision.BarcodeDetector;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

@Autonomous(name = "AutonomousMode")
public class AutonomousMode extends LinearOpMode {

    public static BarcodeDetector pipeline;
    OpenCvCamera webcam;

    @Override
    public void runOpMode() throws InterruptedException {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        pipeline = new BarcodeDetector();
        webcam.setPipeline(pipeline);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam.startStreaming(320,240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int i) {

            }
        });

        waitForStart();

        while(opModeIsActive()){
            if(pipeline.position == BarcodeDetector.BarcodePosition.ONE){
                telemetry.addData("POSITION", pipeline.position);
                telemetry.update();
            }
            if (pipeline.position == BarcodeDetector.BarcodePosition.TWO){
                telemetry.addData("POSITION", pipeline.position);
                telemetry.update();
            }
            if(pipeline.position == BarcodeDetector.BarcodePosition.THREE){
                telemetry.addData("POSITION", pipeline.position);
                telemetry.update();
            }
        }
    }
}


/*
Trajectories.Action[] actions = new Trajectories.Action[]{
        () -> Trajectories.moveForward(20),
        () -> Trajectories.moveForward(20),
        () -> Trajectories.strafeLeft(20)
};

for (Trajectories.Action action : actions) {
    action.move();
}

 */
