package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.TeleOpFieldCentric;
import org.firstinspires.ftc.teamcode.auto.Trajectories;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@TeleOp (name = "TeleOp", group = "Concept")
//@Disabled
public class TeleOpMode extends LinearOpMode {

    TeleOpFieldCentric driver;
    Trajectories trajectories;
    Intake intake;
    Claw claw;
    CarouselMechanism carouselMechanism;
    SlidesTeleOp slides;
    Outtake2 outtake2;

    @Override
    public void runOpMode() {
//        driver = new TeleOpFieldCentric(hardwareMap, new SampleMecanumDrive(hardwareMap), gamepad1);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        intake = new Intake(gamepad1, hardwareMap);
        claw = new Claw(gamepad1, hardwareMap);
        carouselMechanism = new CarouselMechanism(gamepad1, hardwareMap);
        outtake2 = new Outtake2(hardwareMap, new Claw(gamepad1, hardwareMap), new V4B(gamepad1, hardwareMap), new Intake(gamepad1, hardwareMap));

        outtake2.setOuttakePos(Outtake2.outtakePosEnum.BOTTOM_CLOSE);
        outtake2.setOuttakeInstructions(Outtake2.outtakeInstructionsEnum.CLAW_OPEN);

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.dpad_up) {
                outtake2.setOuttakePos(Outtake2.outtakePosEnum.TOP);
                outtake2.setOuttakeInstructions(Outtake2.outtakeInstructionsEnum.CLAW_CLOSED);
            } else if (gamepad1.back){
                outtake2.setOuttakePos(Outtake2.outtakePosEnum.TSE);
                outtake2.setOuttakeInstructions(Outtake2.outtakeInstructionsEnum.CLAW_CLOSED);
            } else if (gamepad1.dpad_down){
                outtake2.setOuttakePos(Outtake2.outtakePosEnum.BOTTOM_OPEN);
                outtake2.setOuttakeInstructions(Outtake2.outtakeInstructionsEnum.CLAW_CLOSED);
            } else if (gamepad1.dpad_right){
                outtake2.setOuttakePos(Outtake2.outtakePosEnum.MID);
                outtake2.setOuttakeInstructions(Outtake2.outtakeInstructionsEnum.CLAW_CLOSED);
            } else if (gamepad1.y) {
                outtake2.setOuttakePos(Outtake2.outtakePosEnum.TSE_OPEN);
                outtake2.setOuttakeInstructions(Outtake2.outtakeInstructionsEnum.CLAW_CLOSED);
            }

            if (gamepad1.left_bumper) {
                intake.intake_motor.setPower(1);
            } else if (gamepad1.right_bumper) {
                intake.intake_motor.setPower(-1);
            }

            if (gamepad1.right_trigger > 0.75) {
                outtake2.setOuttakePos(Outtake2.outtakePosEnum.BOTTOM_CLOSE);
                outtake2.setOuttakeInstructions(Outtake2.outtakeInstructionsEnum.CLAW_OPEN);
            }

//            driver.move();
            drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x,
                            -gamepad1.right_stick_x
                    )
            );

            drive.update();

            if (gamepad1.x) {
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
