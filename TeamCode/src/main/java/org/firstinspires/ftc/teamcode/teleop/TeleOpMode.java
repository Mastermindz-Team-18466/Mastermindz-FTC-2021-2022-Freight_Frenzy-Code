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
    Outtake outtake;
    Outtake2 outtake2;

    @Override
    public void runOpMode() {
//        driver = new TeleOpFieldCentric(hardwareMap, new SampleMecanumDrive(hardwareMap), gamepad1);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        intake = new Intake(gamepad1, hardwareMap);
        claw = new Claw(gamepad1, hardwareMap);
        carouselMechanism = new CarouselMechanism(gamepad1, hardwareMap);
        outtake = new Outtake(new SlidesTeleOp(gamepad1, hardwareMap), new V4B(gamepad1, hardwareMap), new Claw(gamepad1, hardwareMap), gamepad1);
        outtake2 = new Outtake2(hardwareMap, new Claw(gamepad1, hardwareMap), new V4B(gamepad1, hardwareMap));

        outtake.set(Outtake.Position.BACK);

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.b) {
                outtake2.setTargetLiftPos(Outtake2.liftPos.BOTTOM);
                outtake2.setOuttakePos(Outtake2.outtakePosEnum.CLOSE);
                outtake2.setOuttakeInstructions(Outtake2.outtakeInstructionsEnum.CLAW_OPEN);
            } else if (gamepad1.dpad_down) {
                outtake.set(Outtake.Position.BOTTOM_FORWARD);
            } else if (gamepad1.dpad_up) {
                outtake2.setTargetLiftPos(Outtake2.liftPos.TOP);
                outtake2.setOuttakePos(Outtake2.outtakePosEnum.OPEN);
                outtake2.setOuttakeInstructions(Outtake2.outtakeInstructionsEnum.CLAW_CLOSED);
            }

            if (gamepad1.left_bumper) {
                intake.intake_motor.setPower(0.7);
            } else if (gamepad1.right_bumper) {
                intake.intake_motor.setPower(-0.7);
            } else {
                intake.intake_motor.setPower(0);
            }

            if (gamepad1.right_trigger > 0.75) {
                claw.control(Claw.State.OPEN);
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

            telemetry.update();
            outtake2.update();
        }

        telemetry.addData(">", "Done");
        telemetry.update();



    }
}
