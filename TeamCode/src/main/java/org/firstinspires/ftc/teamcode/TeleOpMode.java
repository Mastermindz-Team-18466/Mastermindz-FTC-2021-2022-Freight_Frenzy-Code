package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Concept: Main", group = "Concept")
//@Disabled
public class TeleOpMode extends LinearOpMode {

    SlidesPID slides;
    FieldOrientedDrive driver;
    Trajectories trajectories;
    Intake intake;
    CarouselMechanism carouselMechanism;

    @Override
    public void runOpMode() {
        driver = new FieldOrientedDrive(gamepad1);
        intake = new Intake(gamepad1, 1);
        carouselMechanism = new CarouselMechanism(gamepad1);

        waitForStart();

        while (opModeIsActive()) {
            intake.control();

            driver.move();

            slides.control();

            carouselMechanism.control();

            Trajectories.Action[] actions = new Trajectories.Action[]{
                    () -> Trajectories.moveForward(100),
                    () -> Trajectories.moveForward(100),
                    () -> Trajectories.strafeLeft(50)
            };

            for (Trajectories.Action action : actions) {
                action.move();
            }


            telemetry.addData("Vector X", driver.vector.getX());
            telemetry.addData("Vector Y", driver.vector.getY());
            telemetry.addData("Vector Direction", driver.vector.getDir());
            telemetry.update();

            driver.finish();
        }

        telemetry.addData(">", "Done");
        telemetry.update();

    }
}
