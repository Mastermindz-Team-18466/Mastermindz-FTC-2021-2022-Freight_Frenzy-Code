package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.PoseStorage;
import org.firstinspires.ftc.teamcode.auto.Trajectories;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@TeleOp (name = "TeleOp", group = "Concept")
//@Disabled
public class TeleOpMode extends LinearOpMode {

    enum Mode {
        DRIVER_CONTROL,
        AUTONOMOUS_CONTROL,
    }

    Mode currentMode = Mode.DRIVER_CONTROL;

    TeleOpFieldCentric driver;
    Trajectories trajectories;
    Intake intake;
    Claw claw;
    CarouselMechanism carouselMechanism;
    SlidesTeleOp slides;
    Outtake2 outtake2;

    Pose2d startPose = new Pose2d(10, -66, Math.toRadians(90));
    Pose2d hubPose =  new Pose2d(-2, -50, Math.toRadians(-68));

    Vector2d startVector = new Vector2d(10, -66);
    Vector2d hubVector = new Vector2d(-2, -50);
    Vector2d warehouse = new Vector2d(50, -62.5);
    Pose2d warehousePose = new Pose2d(50, -62.5, Math.toRadians(0));

    @Override
    public void runOpMode() {
        TeleOpFieldCentric driver = new TeleOpFieldCentric(hardwareMap, new SampleMecanumDrive(hardwareMap), gamepad1);
        driver.drive.setPoseEstimate(warehousePose);

        intake = new Intake(gamepad1, hardwareMap);
        claw = new Claw(gamepad1, hardwareMap);
        carouselMechanism = new CarouselMechanism(gamepad1, hardwareMap);
        outtake2 = new Outtake2(hardwareMap, new Claw(gamepad1, hardwareMap), new V4B(gamepad1, hardwareMap));

        outtake2.setOuttakePos(Outtake2.outtakePosEnum.BOTTOM_CLOSE);
        outtake2.setOuttakeInstructions(Outtake2.outtakeInstructionsEnum.CLAW_OPEN);

        waitForStart();

        while (opModeIsActive()) {
            driver.drive.update();
            Pose2d poseEstimate = driver.drive.getPoseEstimate();

            switch (currentMode){
                case DRIVER_CONTROL:
                    driver.move();
                    if (gamepad1.b || gamepad1.x) {
                        currentMode = Mode.AUTONOMOUS_CONTROL;
                    }
                    break;
                case AUTONOMOUS_CONTROL:
                    if (gamepad1.b) {
                        TrajectorySequence traj1 = driver.drive.trajectorySequenceBuilder(poseEstimate)
                                .setReversed(true)
                                .splineTo(hubVector, Math.toRadians(-68+180))
                                .build();

                        driver.drive.followTrajectorySequenceAsync(traj1);

                        if (!driver.drive.isBusy()) {
                            currentMode = Mode.DRIVER_CONTROL;
                        }
                    }

                    else if (gamepad1.x) {
                        TrajectorySequence traj2 = driver.drive.trajectorySequenceBuilder(poseEstimate)
                                .setReversed(false)
                                .splineTo(warehouse, Math.toRadians(0))
                                .build();

                        driver.drive.followTrajectorySequenceAsync(traj2);

                        if (!driver.drive.isBusy()) {
                            currentMode = Mode.DRIVER_CONTROL;
                        }
                    }
//                    currentMode = Mode.DRIVER_CONTROL;
                    if (!driver.drive.isBusy()) {
                        currentMode = Mode.DRIVER_CONTROL;
                    }
            }

            if(gamepad1.y){
                driver.drive.setPoseEstimate(new Pose2d(12.5, -62.5, Math.toRadians(0)));
            }

            if (gamepad2.dpad_up) {
                outtake2.setOuttakePos(Outtake2.outtakePosEnum.TOP);
                outtake2.setOuttakeInstructions(Outtake2.outtakeInstructionsEnum.CLAW_CLOSED);
            } else if (gamepad2.back){
                outtake2.setOuttakePos(Outtake2.outtakePosEnum.TSE);
                outtake2.setOuttakeInstructions(Outtake2.outtakeInstructionsEnum.CLAW_CLOSED);
            } else if (gamepad2.dpad_down){
                outtake2.setOuttakePos(Outtake2.outtakePosEnum.BOTTOM_OPEN);
                outtake2.setOuttakeInstructions(Outtake2.outtakeInstructionsEnum.CLAW_CLOSED);
            } else if (gamepad2.dpad_right){
                outtake2.setOuttakePos(Outtake2.outtakePosEnum.MID);
                outtake2.setOuttakeInstructions(Outtake2.outtakeInstructionsEnum.CLAW_CLOSED);
            } else if (gamepad2.y) {
                outtake2.setOuttakePos(Outtake2.outtakePosEnum.TSE_OPEN);
                outtake2.setOuttakeInstructions(Outtake2.outtakeInstructionsEnum.CLAW_CLOSED);
            }

            if (gamepad2.left_bumper) {
                intake.intake_motor.setPower(1);
            } else if (gamepad2.right_bumper) {
                intake.intake_motor.setPower(-1);
            } else{
                intake.intake_motor.setPower(0);
            }

            if (gamepad2.right_trigger > 0.75) {
                outtake2.setOuttakePos(Outtake2.outtakePosEnum.BOTTOM_CLOSE);
                outtake2.setOuttakeInstructions(Outtake2.outtakeInstructionsEnum.CLAW_OPEN);
            }


            if (gamepad2.x) {
                carouselMechanism.start();
            } else {
                carouselMechanism.stop();
            }

            telemetry.addData("IntakePower", intake.intake_motor.getPower());
            telemetry.update();
            outtake2.update();
        }

        telemetry.addData(">", "Done");
        telemetry.update();



    }
}
