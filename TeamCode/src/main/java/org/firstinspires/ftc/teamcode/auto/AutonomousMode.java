package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.TeleOpFieldCentric;
import org.firstinspires.ftc.teamcode.auto.Trajectories;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.teleop.CarouselMechanism;
import org.firstinspires.ftc.teamcode.teleop.Claw;
import org.firstinspires.ftc.teamcode.teleop.Intake;
import org.firstinspires.ftc.teamcode.teleop.Outtake2;
import org.firstinspires.ftc.teamcode.teleop.SlidesTeleOp;
import org.firstinspires.ftc.teamcode.teleop.V4B;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.vision.BarcodeDetector;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import com.acmerobotics.roadrunner.geometry.Vector2d;


@Autonomous(name = "AutonomousMode", group = "Concept")
//@Disabled
public class AutonomousMode extends LinearOpMode {

    public static BarcodeDetector pipeline;
    OpenCvCamera webcam;
    Intake intake;
    Claw claw;
    CarouselMechanism carouselMechanism;
    Outtake2 outtake2;
    BarcodeDetector.BarcodePosition tsePosition;
    TeleOpFieldCentric driver;
    Trajectories trajectories;
    SlidesTeleOp slides;

    @Override
    public void runOpMode() {
//        driver = new TeleOpFieldCentric(hardwareMap, new SampleMecanumDrive(hardwareMap), gamepad1);

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

        Pose2d startPose = new Pose2d(10, -66, Math.toRadians(90));
        Pose2d hubPose = new Pose2d(-2, -50, Math.toRadians(-68));
        Vector2d hubVector = new Vector2d(-2, -50);
        Vector2d warehouse = new Vector2d(46, -62.5);
        drive.setPoseEstimate(startPose);

        TrajectorySequence trajStart = drive.trajectorySequenceBuilder(startPose)
                .UNSTABLE_addTemporalMarkerOffset(1.3, () -> {

                    if (tsePosition == BarcodeDetector.BarcodePosition.ONE) {
                        outtake2.setOuttakePos(Outtake2.outtakePosEnum.BOTTOM_OPEN);
                        outtake2.setOuttakeInstructions(Outtake2.outtakeInstructionsEnum.CLAW_CLOSED);
                    } else if (tsePosition == BarcodeDetector.BarcodePosition.TWO) {
                        outtake2.setOuttakePos(Outtake2.outtakePosEnum.MID);
                        outtake2.setOuttakeInstructions(Outtake2.outtakeInstructionsEnum.CLAW_CLOSED);
                    } else if (tsePosition == BarcodeDetector.BarcodePosition.THREE) {
                        outtake2.setOuttakePos(Outtake2.outtakePosEnum.TOP);
                        outtake2.setOuttakeInstructions(Outtake2.outtakeInstructionsEnum.CLAW_CLOSED);
                    }

                })

                .UNSTABLE_addTemporalMarkerOffset(2.2, () -> {
                    outtake2.setOuttakePos(Outtake2.outtakePosEnum.BOTTOM_CLOSE);
                    outtake2.setOuttakeInstructions(Outtake2.outtakeInstructionsEnum.CLAW_OPEN);
                })
                .waitSeconds(0.5)
                .lineToLinearHeading(hubPose)
                .waitSeconds(0.2)
                .build();


        outtake2.setOuttakePos(Outtake2.outtakePosEnum.BOTTOM_CLOSE);
        outtake2.setOuttakeInstructions(Outtake2.outtakeInstructionsEnum.CLAW_OPEN);

        waitForStart();

        drive.followTrajectorySequenceAsync(trajStart);

        int cycles = 0;
        long startTime = System.currentTimeMillis();
        while (opModeIsActive()) {
            tsePosition = pipeline.scanBarcode();
            long currentTime = System.currentTimeMillis();


            if (currentTime - startTime >= 2400 && !drive.isBusy() && cycles < 5) {
                Pose2d currentPose = drive.getPoseEstimate();
                drive.followTrajectorySequenceAsync(drive.trajectorySequenceBuilder(currentPose)
                        .UNSTABLE_addTemporalMarkerOffset(3, () -> {
                            outtake2.setOuttakePos(Outtake2.outtakePosEnum.TOP);
                            outtake2.setOuttakeInstructions(Outtake2.outtakeInstructionsEnum.CLAW_CLOSED);
                        })
                        .UNSTABLE_addTemporalMarkerOffset(5, () -> {
                            outtake2.setOuttakePos(Outtake2.outtakePosEnum.BOTTOM_CLOSE);
                            outtake2.setOuttakeInstructions(Outtake2.outtakeInstructionsEnum.CLAW_OPEN);
                        })
                        .splineTo(warehouse, Math.toRadians(0))
                        .waitSeconds(0.1)
                        .setReversed(true)
                        .splineTo(hubVector, Math.toRadians(-68 + 180))
                        .build()
                );
                cycles++;
            }


            drive.update();

            if (gamepad1.x) {
                carouselMechanism.start();
            } else {
                carouselMechanism.stop();
            }


            telemetry.addData("IntakePower", intake.intake_motor.getPower());
            telemetry.addData("cycle", cycles);
            telemetry.addData("POSITION", pipeline.position);
            telemetry.update();
            outtake2.update();
        }

        telemetry.addData(">", "Done");
        telemetry.update();


    }
}
