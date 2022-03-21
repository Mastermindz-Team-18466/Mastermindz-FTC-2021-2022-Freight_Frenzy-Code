package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.checkerframework.checker.units.qual.C;
import org.firstinspires.ftc.teamcode.auto.Trajectories;

@TeleOp (name = "TeleOp", group = "Concept")
//@Disabled
public class TeleOpMode extends LinearOpMode {

    FieldOrientatedDrive driver;
    Trajectories trajectories;
    Intake intake;
    Claw claw;
    CarouselMechanism carouselMechanism;
    SlidesTeleOp slides;
    Outtake outtake;

    @Override
    public void runOpMode() {
        driver = new FieldOrientatedDrive(gamepad1, hardwareMap);
        intake = new Intake(gamepad1, hardwareMap);
        claw = new Claw(gamepad1, hardwareMap);
        carouselMechanism = new CarouselMechanism(gamepad1, hardwareMap);
        outtake = new Outtake(new SlidesTeleOp(gamepad1, hardwareMap), new V4B(gamepad1, hardwareMap), new Claw(gamepad1, hardwareMap), gamepad1);

        outtake.set(Outtake.Position.BACK);

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.b) {
                outtake.set(Outtake.Position.BACK);
            } else if (gamepad1.dpad_down) {
                outtake.set(Outtake.Position.BOTTOM_FORWARD);
            } else if (gamepad1.dpad_up) {
                outtake.set(Outtake.Position.TOP_FORWARD);
            }

            if (gamepad1.left_bumper) {
                intake.intake_motor.setPower(2);
            } else if (gamepad1.right_bumper) {
                intake.intake_motor.setPower(-2);
            } else {
                intake.intake_motor.setPower(0);
            }

            if (gamepad1.right_trigger > 0.75) {
                claw.control(Claw.State.OPEN);
            }

            driver.move();

            if (gamepad1.x) {
                carouselMechanism.start();
            } else {
                carouselMechanism.stop();
            }

            /*
            Trajectories.Action[] actions = new Trajectories.Action[]{
                    () -> Trajectories.moveForward(100),
                    () -> Trajectories.moveForward(100),
                    () -> Trajectories.strafeLeft(50)
            };

            for (Trajectories.Action action : actions) {
                action.move();
            }
            */

            telemetry.update();
        }

        telemetry.addData(">", "Done");
        telemetry.update();

    }
}
