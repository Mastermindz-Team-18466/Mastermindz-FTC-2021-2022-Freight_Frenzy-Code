package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

@TeleOp(name = "Concept: Main,", group = "Concept")
//@Disabled
public class Main extends LinearOpMode {

    SlidesPID slides;
    FieldOrientedDriveV2 driver;
    Trajectories trajectories;
    Gamepad gamepad;

    @Override
    public void runOpMode() {
        driver = new FieldOrientedDriveV2();

        waitForStart();

        while (opModeIsActive()) {
            driver.move();

            slides.control();

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
