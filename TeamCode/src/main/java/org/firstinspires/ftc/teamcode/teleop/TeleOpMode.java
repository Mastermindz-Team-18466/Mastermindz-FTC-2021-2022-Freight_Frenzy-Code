package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.auto.SlidesPID;
import org.firstinspires.ftc.teamcode.auto.Trajectories;

@TeleOp (name = "TeleOp", group = "Concept")
//@Disabled
public class TeleOpMode extends LinearOpMode {

    FieldOrientatedDrive driver;
    Trajectories trajectories;
    Intake intake;
    CarouselMechanism carouselMechanism;
    SlidesTeleOp slides;

    @Override
    public void runOpMode() {
        driver = new FieldOrientatedDrive(gamepad1);
        intake = new Intake(gamepad1, 1);
        carouselMechanism = new CarouselMechanism(gamepad1);
        slides = new SlidesTeleOp(gamepad1);

        waitForStart();

        while (opModeIsActive()) {
            intake.control();

            slides.control();

            driver.move();

            carouselMechanism.control();

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
