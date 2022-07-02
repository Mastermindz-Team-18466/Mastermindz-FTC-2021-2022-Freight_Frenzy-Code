package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.teleop.CarouselMechanism;
import org.firstinspires.ftc.teamcode.teleop.Claw;
import org.firstinspires.ftc.teamcode.teleop.Intake;
import org.firstinspires.ftc.teamcode.teleop.Outtake;
import org.firstinspires.ftc.teamcode.teleop.Outtake2;
import org.firstinspires.ftc.teamcode.teleop.SlidesTeleOp;
import org.firstinspires.ftc.teamcode.teleop.V4B;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.vision.BarcodeDetector;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous(name = "AutonomousMode")
public class AutonomousMode extends LinearOpMode {

    public static BarcodeDetector pipeline;
    OpenCvCamera webcam;
    Intake intake;
    Claw claw;
    CarouselMechanism carouselMechanism;
    Outtake2 outtake2;
    BarcodeDetector.BarcodePosition tsePosition;

    @Override
    public void runOpMode() throws InterruptedException {

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        intake = new Intake(gamepad1, hardwareMap);
        claw = new Claw(gamepad1, hardwareMap);
        carouselMechanism = new CarouselMechanism(gamepad1, hardwareMap);
        outtake2 = new Outtake2(hardwareMap, new Claw(gamepad1, hardwareMap), new V4B(gamepad1, hardwareMap), new Intake(gamepad1, hardwareMap));

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        pipeline = new BarcodeDetector();
        webcam.setPipeline(pipeline);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int i) {

            }
        });

        Pose2d startPose = new Pose2d(12, -66, Math.toRadians(90));
        Pose2d hubPose = new Pose2d(-3, -46.5, Math.toRadians(-68));
        Vector2d warehouse = new Vector2d(46, -62.5);
        drive.setPoseEstimate(startPose);


        TrajectorySequence trajStart = drive.trajectorySequenceBuilder(startPose)
                .UNSTABLE_addTemporalMarkerOffset(1, () -> {

                    outtake2.setOuttakePos(Outtake2.outtakePosEnum.MID);
                    outtake2.setOuttakeInstructions(Outtake2.outtakeInstructionsEnum.CLAW_CLOSED);

//                    if (tsePosition == BarcodeDetector.BarcodePosition.ONE) {
//                        outtake2.setOuttakePos(Outtake2.outtakePosEnum.BOTTOM_OPEN);
//                        outtake2.setOuttakeInstructions(Outtake2.outtakeInstructionsEnum.CLAW_CLOSED);
//                    }
//                    else if (tsePosition == BarcodeDetector.BarcodePosition.TWO) {
//                        outtake2.setOuttakePos(Outtake2.outtakePosEnum.MID);
//                        outtake2.setOuttakeInstructions(Outtake2.outtakeInstructionsEnum.CLAW_CLOSED);
//                    }
//                    else if (tsePosition == BarcodeDetector.BarcodePosition.THREE) {
//                        outtake2.setOuttakePos(Outtake2.outtakePosEnum.TOP);
//                        outtake2.setOuttakeInstructions(Outtake2.outtakeInstructionsEnum.CLAW_CLOSED);
//                    }

                })

                .UNSTABLE_addTemporalMarkerOffset(5, () -> {
                    outtake2.setOuttakePos(Outtake2.outtakePosEnum.BOTTOM_CLOSE);
                    outtake2.setOuttakeInstructions(Outtake2.outtakeInstructionsEnum.CLAW_OPEN);
                })
                .waitSeconds(10)
                .build();

        outtake2.setOuttakePos(Outtake2.outtakePosEnum.BOTTOM_CLOSE);
        outtake2.setOuttakeInstructions(Outtake2.outtakeInstructionsEnum.CLAW_OPEN);

        waitForStart();
        if (isStopRequested()) return;


        drive.followTrajectorySequenceAsync(trajStart);


        long startTime = System.currentTimeMillis();
        while (opModeIsActive()) {
            tsePosition = pipeline.scanBarcode();
            telemetry.addData("POSITION", pipeline.position);
            telemetry.update();

            drive.update();
            outtake2.update();
        }
    }
}
