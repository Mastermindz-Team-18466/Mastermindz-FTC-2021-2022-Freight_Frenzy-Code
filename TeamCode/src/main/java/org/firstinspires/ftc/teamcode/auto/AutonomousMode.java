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
import org.firstinspires.ftc.teamcode.teleop.SlidesTeleOp;
import org.firstinspires.ftc.teamcode.teleop.V4B;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.vision.BarcodeDetector;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.openftc.easyopencv.OpenCvInternalCamera;

@Autonomous(name = "AutonomousMode")
public class AutonomousMode extends LinearOpMode {

    public static BarcodeDetector pipeline;
    OpenCvCamera webcam;
    Intake intake;
    Claw claw;
    CarouselMechanism carouselMechanism;
    Outtake outtake;
    BarcodeDetector.BarcodePosition tsePosition;

    @Override
    public void runOpMode() throws InterruptedException {

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        intake = new Intake(gamepad1, hardwareMap);
        claw = new Claw(gamepad1, hardwareMap);
        carouselMechanism = new CarouselMechanism(gamepad1, hardwareMap);
        outtake = new Outtake(new SlidesTeleOp(gamepad1, hardwareMap), new V4B(gamepad1, hardwareMap), new Claw(gamepad1, hardwareMap), gamepad1);

        outtake.set(Outtake.Position.AUTO_START);

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

        Pose2d startPose = new Pose2d(12, -66, Math.toRadians(90));
        Pose2d hubPose = new Pose2d(-3, -46.5, Math.toRadians(-68));
        Vector2d warehouse = new Vector2d(46, -62.5);
        drive.setPoseEstimate(startPose);

        while (!isStarted()) {
            tsePosition = pipeline.scanBarcode();
            telemetry.addData("POSITION", pipeline.position);
            telemetry.update();
        }

        waitForStart();
        if (isStopRequested()) return;

        TrajectorySequence trajStart = drive.trajectorySequenceBuilder(startPose)
                .UNSTABLE_addTemporalMarkerOffset(1.15, ()-> {
                    intake.intake_motor.setPower(-0.5);

                    if (tsePosition == BarcodeDetector.BarcodePosition.ONE) {
                        outtake.set(Outtake.Position.BOTTOM_FORWARD);
                    }
                    if (tsePosition == BarcodeDetector.BarcodePosition.TWO) {
                        outtake.set(Outtake.Position.MIDDLE_FORWARD);
                    }
                    if (tsePosition == BarcodeDetector.BarcodePosition.THREE) {
                        outtake.set(Outtake.Position.TOP_FORWARD);
                    }

                })
                .lineToLinearHeading(hubPose)
                .splineTo(warehouse, Math.toRadians(0))
                .build();



        drive.followTrajectorySequenceAsync(trajStart);


        long startTime = System.currentTimeMillis();
        while (opModeIsActive()) {
//            drive.followTrajectorySequenceAsync(drive.trajectorySequenceBuilder(new Pose2d(11.5, -60, Math.toRadians(90)))
////                        .UNSTABLE_addTemporalMarkerOffset(0, () -> {
////                            intake.intake_motor.setPower(0.7);
////                        })
////                        .UNSTABLE_addTemporalMarkerOffset(0.95, () -> {
////                            if (tsePosition == BarcodeDetector.BarcodePosition.ONE) {
////                                telemetry.addData("POSITION", pipeline.position);
////                                telemetry.update();
////                            }
////                            if (tsePosition == BarcodeDetector.BarcodePosition.TWO) {
////                                telemetry.addData("POSITION", pipeline.position);
////                                telemetry.update();
////                            }
////                            if (tsePosition == BarcodeDetector.BarcodePosition.THREE) {
////                                telemetry.addData("POSITION", pipeline.position);
////                                telemetry.update();
////                            }
////                        })
//                    .lineToLinearHeading(new Pose2d(-1, -50, Math.toRadians(-68)))
//                    .splineTo(new Vector2d(46, -62.5), Math.toRadians(0))
//                    .setReversed(true)
//                    .splineTo(new Vector2d(-1, -50), Math.toRadians(-68 + 180))
//                    .setReversed(false)
//                    .build());

            drive.update();

        }
    }
}
